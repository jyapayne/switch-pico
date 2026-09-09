#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

// Compile the real patched SDK implementation, not a model of its credit logic.
#include "hci.c"

static uint32_t clock_ms;
static void (*receive_packet)(uint8_t, uint8_t *, uint16_t);
static bool transport_ready;
static unsigned acl_delivered;
static unsigned sco_delivered;
static unsigned wire_count;
static uint8_t wire[32][64];
static int wire_size[32];
static void (*during_send)(void);
static uint8_t wire_type[32];
static bool complete_inline;
static uint8_t *last_transport_packet;

static void transport_sent(void){
    uint8_t sent[] = {HCI_EVENT_TRANSPORT_PACKET_SENT, 0};
    receive_packet(HCI_EVENT_PACKET, sent, sizeof(sent));
}

noreturn void btstack_assert_failed(const char *file, uint16_t line){
    fprintf(stderr, "BTstack assertion: %s:%u\n", file, line);
    abort();
}

static void set_timer(btstack_timer_source_t *timer, uint32_t timeout_ms){
    // Match pico_btstack's millisecond quantization, including its extra tick.
    timer->timeout = clock_ms + timeout_ms + 1;
}

static uint32_t get_time_ms(void){
    return clock_ms;
}

static const btstack_run_loop_t run_loop = {
    .init = btstack_run_loop_base_init,
    .set_timer = set_timer,
    .add_timer = btstack_run_loop_base_add_timer,
    .remove_timer = btstack_run_loop_base_remove_timer,
    .get_time_ms = get_time_ms,
};

static void advance(uint32_t ms){
    clock_ms += ms;
    btstack_run_loop_base_process_timers(clock_ms);
}

static void register_receiver(void (*handler)(uint8_t, uint8_t *, uint16_t)){
    receive_packet = handler;
}

static int transport_open(void){
    return 0;
}

static int transport_close(void){
    return 0;
}

static int can_send(uint8_t packet_type){
    (void) packet_type;
    return transport_ready;
}

static int send_packet(uint8_t type, uint8_t *packet, int size){
    assert(type == HCI_COMMAND_DATA_PACKET || type == HCI_ACL_DATA_PACKET);
    assert(wire_count < 32 && size <= 64);
    // CYW43 writes its header before the packet and reads word-rounded data.
    assert(((uintptr_t)packet & 3u) == 0);
    memset(packet - HCI_OUTGOING_PRE_BUFFER_SIZE, 0xa5, HCI_OUTGOING_PRE_BUFFER_SIZE);
    wire_type[wire_count] = type;
    last_transport_packet = packet;
    memcpy(wire[wire_count], packet, (size_t) size);
    wire_size[wire_count++] = size;
    if (during_send != NULL){
        void (*callback)(void) = during_send;
        during_send = NULL;
        callback();
    }
    if (complete_inline) transport_sent();
    return 0;
}

static hci_transport_t transport = {
    .name = "credit-regression",
    .open = transport_open,
    .close = transport_close,
    .register_packet_handler = register_receiver,
    .send_packet = send_packet,
};

static void on_acl(uint8_t type, uint16_t channel, uint8_t *packet, uint16_t size){
    (void) channel;
    assert(type == HCI_ACL_DATA_PACKET && size == 9 && packet[8] == 0x5a);
    acl_delivered++;
}

static void on_sco(uint8_t type, uint16_t channel, uint8_t *packet, uint16_t size){
    (void) channel;
    assert(type == HCI_SCO_DATA_PACKET && size == 4 && packet[3] == 0x5a);
    sco_delivered++;
}

static void working(void){
    // Fixture bypasses controller initialization, leaving the production receive/run paths intact.
    hci_stack->state = HCI_STATE_WORKING;
    hci_stack->gap_tasks_classic = 0;
    hci_stack->le_scanning_param_update = false;
    hci_stack->num_cmd_packets = 1;
    hci_register_acl_packet_handler(on_acl);
    hci_register_sco_packet_handler(on_sco);
}

static void begin(uint32_t now, bool asynchronous){
    clock_ms = now;
    transport_ready = true;
    transport.can_send_packet_now = asynchronous ? can_send : NULL;
    during_send = NULL;
    complete_inline = false;
    last_transport_packet = NULL;
    wire_count = acl_delivered = sco_delivered = 0;
    btstack_run_loop_init(&run_loop);
    btstack_memory_init();
    hci_init(&transport, NULL);
    working();
}

static void finish(void){
    if (hci_stack != NULL){
        // Isolate close's timer cancellation from the unrelated asynchronous shutdown FSM.
        hci_stack->state = HCI_STATE_OFF;
        hci_close();
    }
    advance(10);
    btstack_memory_deinit();
    btstack_run_loop_deinit();
}

static void add_connection(uint16_t handle, bd_addr_type_t type){
    hci_connection_t *connection = btstack_memory_hci_connection_get();
    assert(connection != NULL);
    connection->con_handle = handle;
    connection->address_type = type;
    hci_connection_init(connection);
    connection->state = OPEN;
    btstack_linked_list_add_tail(&hci_stack->connections, (btstack_linked_item_t *) connection);
}

static void acl(uint16_t handle){
    uint8_t packet[] = {0, 0x20, 5, 0, 1, 0, 0x40, 0, 0x5a};
    little_endian_store_16(packet, 0, handle | 0x2000);
    receive_packet(HCI_ACL_DATA_PACKET, packet, sizeof(packet));
}

static void disconnected(uint16_t handle){
    uint8_t packet[] = {HCI_EVENT_DISCONNECTION_COMPLETE, 4, 0, 0, 0, 0x13};
    little_endian_store_16(packet, 3, handle);
    receive_packet(HCI_EVENT_PACKET, packet, sizeof(packet));
}

static void expect_credits(unsigned index, uint16_t handle, uint16_t count){
    const uint8_t *packet = wire[index];
    assert(wire_size[index] == 8);
    assert(packet[0] == 0x35 && packet[1] == 0x0c && packet[2] == 5 && packet[3] == 1);
    assert(little_endian_read_16(packet, 4) == handle);
    assert(little_endian_read_16(packet, 6) == count);
}

#ifdef SWITCH_PICO_HCI_CREDIT_BATCH
static void test_batch_and_ordinary_command(void){
    begin(100, false);
    add_connection(0x41, BD_ADDR_TYPE_ACL);
    acl(0x41);
    assert(acl_delivered == 1 && wire_count == 0);
    gap_set_class_of_device(0x010203);
    assert(wire_count == 1 && little_endian_read_16(wire[0], 0) == HCI_OPCODE_HCI_WRITE_CLASS_OF_DEVICE);
    // Completed packets remain sendable with zero command credits.
    assert(!hci_can_send_command_packet_now());
    acl(0x41);
    assert(acl_delivered == 2 && wire_count == 2);
    expect_credits(1, 0x41, 2);
    advance(2);
    assert(wire_count == 2);
    finish();
}

static void test_deadline_and_retry(void){
    begin(UINT32_MAX - 1, true);
    add_connection(0x41, BD_ADDR_TYPE_ACL);
    acl(0x41);
    advance(1);
    assert(wire_count == 0);
    // Repeated POWER_ON must leave the original deadline intact.
    hci_power_control(HCI_POWER_ON);
    transport_ready = false;
    advance(1);
    assert(wire_count == 0);
    transport_ready = true;
    // No incoming event is needed to retry after a busy transport.
    advance(2);
    assert(wire_count == 1);
    expect_credits(0, 0x41, 1);
    uint8_t sent[] = {HCI_EVENT_TRANSPORT_PACKET_SENT, 0};
    receive_packet(HCI_EVENT_PACKET, sent, sizeof(sent));
    advance(2);
    assert(wire_count == 1);
    finish();
}

static void test_multiple_handles_and_disconnect(void){
    begin(100, false);
    add_connection(0x41, BD_ADDR_TYPE_ACL);
    add_connection(0x42, BD_ADDR_TYPE_ACL);
    acl(0x41);
    acl(0x42);
    assert(wire_count == 1 && wire_size[0] == 12);
    const uint8_t expected[] = {0x35, 0x0c, 9, 2, 0x41, 0, 1, 0, 0x42, 0, 1, 0};
    assert(memcmp(wire[0], expected, sizeof(expected)) == 0);
    acl(0x41);
    advance(1);
    disconnected(0x41);
    add_connection(0x41, BD_ADDR_TYPE_ACL);
    acl(0x41);
    advance(1);
    assert(wire_count == 1); // Removed handle's deadline cannot flush its replacement early.
    advance(1);
    assert(wire_count == 2);
    expect_credits(1, 0x41, 1);
    acl(0x42);
    disconnected(0x42);
    advance(2);
    assert(wire_count == 2); // No empty completion command after the last pending handle disappears.
    finish();
}

static void test_sco_and_malformed_acl(void){
    begin(100, false);
    add_connection(0x41, BD_ADDR_TYPE_ACL);
    add_connection(0x42, BD_ADDR_TYPE_SCO);
    uint8_t sco[] = {0x42, 0, 1, 0x5a};
    receive_packet(HCI_SCO_DATA_PACKET, sco, sizeof(sco));
    assert(sco_delivered == 1 && wire_count == 1);
    expect_credits(0, 0x42, 1);
    // An orphan continuation is counted by HCI before parsing rejects it.
    uint8_t malformed[] = {0x41, 0x10, 1, 0, 0x5a};
    receive_packet(HCI_ACL_DATA_PACKET, malformed, sizeof(malformed));
    receive_packet(HCI_ACL_DATA_PACKET, malformed, sizeof(malformed));
    assert(acl_delivered == 0 && wire_count == 2);
    expect_credits(1, 0x41, 2);
    finish();
}

static void receive_during_send(void){
    acl(0x41);
}

static void replace_stack_during_send(void){
    hci_stack->state = HCI_STATE_OFF;
    hci_close();
    hci_init(&transport, NULL);
    working();
    hci_reserve_packet_buffer();
}

static void test_synchronous_callbacks(void){
    begin(100, false);
    add_connection(0x41, BD_ADDR_TYPE_ACL);
    acl(0x41);
    during_send = receive_during_send;
    advance(2);
    assert(acl_delivered == 2 && wire_count == 1);
    expect_credits(0, 0x41, 1);
    advance(2);
    assert(wire_count == 2);
    expect_credits(1, 0x41, 1); // Incoming callback's new count/timer survived the previous send.
    acl(0x41);
    during_send = replace_stack_during_send;
    advance(2);
    assert(wire_count == 3 && !hci_can_send_command_packet_now());
    // The returning old send must not release the new stack's reserved buffer.
    hci_release_packet_buffer();
    advance(2);
    assert(wire_count == 3);
    finish();
}

static void test_lifecycle(void){
    begin(100, false);
    add_connection(0x41, BD_ADDR_TYPE_ACL);
    acl(0x41);
    hci_stack->state = HCI_STATE_OFF;
    hci_close();
    advance(2);
    assert(wire_count == 0);
    hci_init(&transport, NULL);
    working();
    add_connection(0x42, BD_ADDR_TYPE_ACL);
    acl(0x42);
    // Re-init cancels before memset; preserve connection allocation for explicit cleanup.
    hci_connection_t *old_connection = hci_connection_for_handle(0x42);
    hci_init(&transport, NULL);
    btstack_memory_hci_connection_free(old_connection);
    working();
    advance(2);
    assert(wire_count == 0);
    add_connection(0x43, BD_ADDR_TYPE_ACL);
    acl(0x43);
    old_connection = hci_connection_for_handle(0x43);
    hci_deinit();
    btstack_memory_hci_connection_free(old_connection);
    advance(2);
    assert(wire_count == 0);
    btstack_memory_deinit();
    btstack_run_loop_deinit();

    begin(100, false);
    add_connection(0x44, BD_ADDR_TYPE_ACL);
    acl(0x44);
    hci_power_control(HCI_POWER_SLEEP);
    unsigned after_power_transition = wire_count;
    advance(2);
    assert(wire_count == after_power_transition);
    finish();
}

#ifdef SWITCH_PICO_HCI_CREDIT_BUFFER
static void test_credits_bypass_fragmented_acl(unsigned completion_mode){
    begin(100, completion_mode != 0);
    complete_inline = completion_mode == 2;
    add_connection(0x41, BD_ADDR_TYPE_ACL);
    add_connection(0x42, BD_ADDR_TYPE_LE_PUBLIC);
    hci_stack->acl_packets_total_num = 1;
    hci_stack->acl_data_packet_length = 8;
    hci_reserve_packet_buffer();
    uint8_t *packet = hci_get_outgoing_packet_buffer();
    little_endian_store_16(packet, 0, 0x2041);
    little_endian_store_16(packet, 2, 16);
    for (unsigned i = 0; i < 16; ++i) packet[4 + i] = (uint8_t)(0x80 + i);
    assert(hci_send_acl_packet_buffer(20) == ERROR_CODE_SUCCESS);
    assert(wire_count == 1 && wire_type[0] == HCI_ACL_DATA_PACKET);

    if (completion_mode == 1) transport_ready = false;
    acl(0x42);
    acl(0x42);
    if (completion_mode == 1){
        advance(2);
        assert(wire_count == 1);
        transport_ready = true;
        transport_sent();
    }
    assert(wire_count == 2 && wire_type[1] == HCI_COMMAND_DATA_PACKET);
    expect_credits(1, 0x42, 2);
    if (completion_mode == 1) transport_sent();
    // A credit completion must not release the still-prepared ACL continuation.
    assert(!hci_can_send_command_packet_now());

    uint8_t completed[] = {HCI_EVENT_NUMBER_OF_COMPLETED_PACKETS, 5, 1, 0x41, 0, 1, 0};
    receive_packet(HCI_EVENT_PACKET, completed, sizeof(completed));
    assert(wire_count == 3 && wire_type[2] == HCI_ACL_DATA_PACKET);
    assert(wire_size[0] == 12 && wire_size[2] == 12);
    assert(little_endian_read_16(wire[2], 0) == 0x1041);
    for (unsigned i = 0; i < 8; ++i){
        assert(wire[0][4 + i] == 0x80 + i);
        assert(wire[2][4 + i] == 0x88 + i);
    }
    if (completion_mode == 1) transport_sent();
    assert(hci_can_send_command_packet_now());
    finish();
}

static void test_credit_buffer_async_ownership(void){
    begin(100, true);
    add_connection(0x41, BD_ADDR_TYPE_ACL);
    hci_reserve_packet_buffer();
    acl(0x41);
    advance(2);
    assert(wire_count == 1);
    uint8_t *in_flight = last_transport_packet;
    uint8_t saved[8];
    memcpy(saved, in_flight, sizeof(saved));
    acl(0x41);
    advance(2);
    assert(wire_count == 1 && memcmp(saved, in_flight, sizeof(saved)) == 0);
    // Even a transport that reports ready must not permit overlapping sends.
    assert(!hci_can_send_prepared_acl_packet_now(0x41));
    transport_sent();
    assert(wire_count == 2);
    expect_credits(1, 0x41, 1);
    transport_sent();
    assert(!hci_can_send_command_packet_now());
    hci_release_packet_buffer();
    assert(hci_can_send_command_packet_now());
    finish();
}

static void sleep_during_send(void){
    hci_power_control(HCI_POWER_SLEEP);
}

static void test_credit_completion_during_sleep(void){
    begin(100, false);
    add_connection(0x41, BD_ADDR_TYPE_ACL);
    acl(0x41);
    during_send = sleep_during_send;
    advance(2);
    assert(wire_count == 1);
    expect_credits(0, 0x41, 1);
    // Cancelling batching must not strand ownership of a completed synchronous send.
    assert(hci_can_send_command_packet_now());
    finish();

    begin(100, true);
    add_connection(0x41, BD_ADDR_TYPE_ACL);
    hci_reserve_packet_buffer();
    acl(0x41);
    advance(2);
    hci_power_control(HCI_POWER_SLEEP);
    transport_sent();
    // A late credit completion after sleep still must not release another packet.
    assert(!hci_can_send_command_packet_now());
    hci_release_packet_buffer();
    assert(hci_can_send_command_packet_now());
    finish();
}

static void test_credit_buffer_power_cycle(void){
    begin(100, true);
    add_connection(0x41, BD_ADDR_TYPE_ACL);
    acl(0x41);
    advance(2);
    assert(wire_count == 1);
    // A wedged transport never completes the credit send; shutdown closes it.
    transport_ready = false;
    hci_power_control(HCI_POWER_OFF);
    advance(1001);
    transport_ready = true;
    hci_power_control(HCI_POWER_ON);
    assert(wire_count == 2);
    assert(wire_type[1] == HCI_COMMAND_DATA_PACKET);
    assert(little_endian_read_16(wire[1], 0) == HCI_OPCODE_HCI_RESET);
    finish();
}
#endif
#endif

int main(void){
#ifdef SWITCH_PICO_HCI_CREDIT_BATCH
    test_batch_and_ordinary_command();
    test_deadline_and_retry();
    test_multiple_handles_and_disconnect();
    test_sco_and_malformed_acl();
    test_synchronous_callbacks();
    test_lifecycle();
#ifdef SWITCH_PICO_HCI_CREDIT_BUFFER
    for (unsigned mode = 0; mode < 3; ++mode) test_credits_bypass_fragmented_acl(mode);
    test_credit_buffer_async_ownership();
    test_credit_completion_during_sleep();
    test_credit_buffer_power_cycle();
#endif
#else
    begin(100, false);
    add_connection(0x41, BD_ADDR_TYPE_ACL);
    acl(0x41);
    assert(acl_delivered == 1 && wire_count == 1);
    expect_credits(0, 0x41, 1);
    acl(0x41);
    assert(acl_delivered == 2 && wire_count == 2);
    expect_credits(1, 0x41, 1);
    advance(2);
    assert(wire_count == 2);
    finish();
#endif
    puts("BTstack credit behavior passed");
    return 0;
}
