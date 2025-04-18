#include "smart_servo.h"
#include "serial.h"
#include "string.h"
#include <unistd.h>


static uint8_t compute_chk(servo_msg_t* msg);

static void set_chk(servo_msg_t* msg, uint8_t chk) {
    *((uint8_t*)msg + msg->len + 3) = chk;
}

static void send_msg(int fd, servo_msg_t* msg) {
    writeData(fd,(uint8_t*)msg, msg->len+4, true);
}



SmartServo::Status SmartServo::readStatus()
{
    size_t n = 0;
    // sync on start bytes 0xFFFF
    servo_status.STX = 0;

    while(servo_status.STX != 0xFFFF) {
        //uint8_t tmp = 0;
        servo_status.STX <<= 8;
        n = ::read(serial_fd, (uint8_t*)&servo_status.STX, 1);
        if(n != 1) { return Status::STATUS_TIMEOUT; }
    }

    servo_status.id = 42;
    servo_status.len = 42;
    servo_status.instruction = 42;

    // Read ID, LEN, ERROR, and either CHK if there is not params, or the first byte of the params.
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wstringop-overflow"    // Trust me, I'm an engineer
    n = ::read(serial_fd, (uint8_t*)&servo_status.id, 4);
    #pragma GCC diagnostic pop
    if (n != 4) { return Status::STATUS_TIMEOUT; }

    if(servo_status.len > 2) {
        // there is more data to be read
        uint8_t* next_data = (uint8_t*)&servo_status.params + 1;
        n = ::read(serial_fd, next_data, servo_status.len-2);
        if (n+2 != servo_status.len) { return Status::STATUS_TIMEOUT; }
    }

    if(compute_chk(&servo_status) != servo_status.params[servo_status.len-2]) {
        return Status::CHECKSUM_ERROR;
    }

    return Status::OK;

}

void SmartServo::flushSerialInput()
{
    tcflush(serial_fd, TCIFLUSH);
}

void SmartServo::init()
{
    //sdStart(sd, &sdconf);
    init_serial(serial_fd, B500000);
}

void SmartServo::setSerialBaudrate(uint32_t speed)
{
    init_serial(serial_fd, speed);
}

SmartServo::Status SmartServo::ping(uint8_t id)
{
    servo_msg.STX = 0xFFFF;
    servo_msg.id = id;
    servo_msg.len = 2;
    servo_msg.instruction = Instruction::SMART_SERVO_PING;
    set_chk(&servo_msg, compute_chk(&servo_msg));

    flushSerialInput();
    send_msg(serial_fd, &servo_msg);

    return readStatus();
}

SmartServo::Status SmartServo::read(record_t *record) {
    servo_msg.STX = 0xFFFF;
    servo_msg.id = record->id;
    servo_msg.len = 4;
    servo_msg.instruction = Instruction::SMART_SERVO_READ;
    servo_msg.params[0] = record->reg;
    servo_msg.params[1] = record->len;
    set_chk(&servo_msg, compute_chk(&servo_msg));

    flushSerialInput();
    send_msg(serial_fd, &servo_msg);

    SmartServo::Status status = readStatus();
    memcpy(&record->data, servo_status.params, record->len);

    return status;
}

SmartServo::Status SmartServo::write(record_t *record, bool is_reg_write) {
    servo_msg.STX = 0xFFFF;
    servo_msg.id = record->id;
    servo_msg.len = record->len + 3;
    if(is_reg_write) {
        servo_msg.instruction = Instruction::SMART_SERVO_REG_WRITE;
    } else {
        servo_msg.instruction = Instruction::SMART_SERVO_WRITE;
    }
    servo_msg.params[0] = record->reg;
    memcpy(&servo_msg.params[1], record->data, record->len);
    set_chk(&servo_msg, compute_chk(&servo_msg));

    flushSerialInput();
    send_msg(serial_fd, &servo_msg);

    return record->id != BROADCAST_ID && response_level == RL_NORMAL ? readStatus(): Status::OK;
}

SmartServo::Status SmartServo::action(uint8_t id) {
    servo_msg.STX = 0xFFFF;
    servo_msg.id = id;
    servo_msg.len = 2;
    servo_msg.instruction = Instruction::SMART_SERVO_ACTION;
    set_chk(&servo_msg, compute_chk(&servo_msg));

    flushSerialInput();
    send_msg(serial_fd, &servo_msg);

    return id != BROADCAST_ID && response_level == RL_NORMAL ? readStatus(): Status::OK;
}

SmartServo::Status SmartServo::reset(uint8_t id) {
    if(id == BROADCAST_ID) {
        // Broadcast ID cannot be use for reset.
        return Status::INVALID_PARAMS;
    }

    servo_msg.STX = 0xFFFF;
    servo_msg.id = id;
    servo_msg.len = 2;
    servo_msg.instruction = Instruction::SMART_SERVO_RESET;
    set_chk(&servo_msg, compute_chk(&servo_msg));

    flushSerialInput();
    send_msg(serial_fd, &servo_msg);

    return response_level == RL_NORMAL ? readStatus(): Status::OK;
}

SmartServo::Status SmartServo::detectBaudrate()
{
    uint32_t baudrates[] = {1000000, 500000, 250000};
    for(auto baud: baudrates) {
        setSerialBaudrate(baud);
        if(ping(BROADCAST_ID) == SmartServo::OK) {
            return SmartServo::OK;
        }
    }
    return Status::STATUS_TIMEOUT;
}

SmartServo::Status SmartServo::sync_write(record_t *records, size_t nb_records) {
    if(nb_records < 1) {
        return Status::INVALID_PARAMS;
    }

    

    uint8_t records_reg = records[0].reg;
    uint8_t records_len = records[0].len;
    
    servo_msg.STX = 0xFFFF;
    servo_msg.id = BROADCAST_ID;
    servo_msg.len = nb_records * (records_len+1) + 4;
    if(servo_msg.len > TX_BUF_LEN) {
        return Status::TX_BUFFER_OVERFLOW;
    }
    servo_msg.instruction = Instruction::SMART_SERVO_SYNC_WRITE;
    servo_msg.params[0] = records_reg;
    servo_msg.params[1] = records_len;

    for(size_t i=0; i<nb_records; i++) {
        if(records[i].len != records_len || records[i].reg != records_reg) {
            // len and reg must be the same in all records
            return Status::INVALID_PARAMS;
        }
        uint8_t* rec_id_p = &servo_msg.params[2] + i * (records_len+1);
        uint8_t* data_p = rec_id_p + 1;
        *rec_id_p = records[i].id;
        memcpy(data_p, records[i].data, records_len);
    }

    set_chk(&servo_msg, compute_chk(&servo_msg));

    flushSerialInput();
    send_msg(serial_fd, &servo_msg);

    // do the servos answer with status packet ???
    return Status::OK;
}

SmartServo::Status SmartServo::writeRegister(uint8_t id, uint8_t reg, uint8_t value)
{
    	SmartServo::record_t rec = {
		.id = id,
		.reg = reg,
		.len = 1,
		.data = {value}
	};

	return write(&rec);
}

static uint8_t compute_chk(servo_msg_t* msg) {
    uint8_t chk = 0;
    size_t chk_len = msg->len + 1;
    uint8_t* data = (uint8_t*) &msg->id;
    for(size_t i=0; i<chk_len; i++) {
        chk += data[i];
    }
    chk = ~chk;
    return chk;
}
