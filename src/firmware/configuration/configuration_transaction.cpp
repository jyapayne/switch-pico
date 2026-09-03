#include "configuration/configuration_transaction.h"
#include <string.h>

#include "configuration/adapter_configuration.h"
ConfigurationTransactionStatus ConfigurationTransaction::begin(
    uint32_t transaction_id, uint16_t schema_version, size_t payload_size,
    uint32_t payload_crc) {
    if (snapshot_.status == ConfigurationTransactionStatus::kReceiving ||
        snapshot_.status == ConfigurationTransactionStatus::kPending) {
        return ConfigurationTransactionStatus::kBusy;
    }
    snapshot_ = {};
    snapshot_.transaction_id = transaction_id;
    if (transaction_id == 0 || payload_size == 0) {
        snapshot_.status = ConfigurationTransactionStatus::kMalformed;
        return snapshot_.status;
    }
    if (schema_version != ADAPTER_CONFIGURATION_SCHEMA_VERSION) {
        snapshot_.status =
            ConfigurationTransactionStatus::kUnsupportedSchema;
        return snapshot_.status;
    }
    if (payload_size > CONFIGURATION_STORAGE_MAX_PAYLOAD_SIZE) {
        snapshot_.status = ConfigurationTransactionStatus::kTooLarge;
        return snapshot_.status;
    }

    snapshot_.expected_size = static_cast<uint16_t>(payload_size);
    snapshot_.expected_crc = payload_crc;
    snapshot_.status = ConfigurationTransactionStatus::kReceiving;
    schema_version_ = schema_version;
    return snapshot_.status;
}

ConfigurationTransactionStatus ConfigurationTransaction::append(
    uint32_t transaction_id, size_t offset, const uint8_t* data,
    size_t size) {
    if (snapshot_.status != ConfigurationTransactionStatus::kReceiving) {
        return ConfigurationTransactionStatus::kBusy;
    }
    if (transaction_id != snapshot_.transaction_id || data == nullptr ||
        size == 0 || offset != snapshot_.received_size ||
        offset + size > snapshot_.expected_size) {
        snapshot_.status = ConfigurationTransactionStatus::kOutOfOrder;
        return snapshot_.status;
    }

    memcpy(&payload_[offset], data, size);
    snapshot_.received_size =
        static_cast<uint16_t>(snapshot_.received_size + size);
    return snapshot_.status;
}

ConfigurationTransactionStatus ConfigurationTransaction::finish(
    uint32_t transaction_id) {
    if (snapshot_.status != ConfigurationTransactionStatus::kReceiving ||
        transaction_id != snapshot_.transaction_id) {
        snapshot_.status = ConfigurationTransactionStatus::kOutOfOrder;
        return snapshot_.status;
    }
    if (snapshot_.received_size != snapshot_.expected_size) {
        snapshot_.status = ConfigurationTransactionStatus::kOutOfOrder;
        return snapshot_.status;
    }
    if (configuration_crc32(payload_, snapshot_.expected_size) !=
        snapshot_.expected_crc) {
        snapshot_.status = ConfigurationTransactionStatus::kBadCrc;
        return snapshot_.status;
    }

    AdapterConfiguration configuration{};
    if (!adapter_configuration_decode(payload_, snapshot_.expected_size,
                                      &configuration)) {
        snapshot_.status = ConfigurationTransactionStatus::kMalformed;
        return snapshot_.status;
    }

    snapshot_.status = ConfigurationTransactionStatus::kPending;
    return snapshot_.status;
}

void ConfigurationTransaction::set_result(
    ConfigurationTransactionStatus status, uint32_t stored_generation,
    uint32_t stored_crc) {
    snapshot_.status = status;
    snapshot_.stored_generation = stored_generation;
    snapshot_.stored_crc = stored_crc;
}

void ConfigurationTransaction::clear() {
    snapshot_ = {};
    schema_version_ = 0;
}

const ConfigurationTransactionSnapshot&
ConfigurationTransaction::snapshot() const {
    return snapshot_;
}

uint16_t ConfigurationTransaction::schema_version() const {
    return schema_version_;
}

const uint8_t* ConfigurationTransaction::payload() const {
    return payload_;
}
