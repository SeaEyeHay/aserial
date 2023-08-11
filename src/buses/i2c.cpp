
/******************************************************************************
 *                  INCLUDES
 *****************************************************************************/

// DEFINITIONS
#include "buses/i2c.hpp"
#include "config.hpp"

// Arduinos
#include <Arduino.h>

// LOGGER
#include <ArduinoLog.h>

/*****************************************************************************/


/******************************************************************************
 *                  LOCAL FUNCTIONS
 *****************************************************************************/

static void masterHandler(void);

static void sendAddr(I2C::Task&);
static void sendReadAddr(I2C::Task&, uint8_t);

static void writeData(I2C::Task&);
static void writeLast(I2C::Task&);
static void readData(I2C::Task&);
static void readLast(I2C::Task&);

static void stop(I2C::Task&);
static void reset(I2C::Task&); 

/*****************************************************************************/


/******************************************************************************
 *                  COMPATIBILITY
 *****************************************************************************/

#ifdef MCU_ARCH_ATMEGA

#endif

 /*****************************************************************************/


 /******************************************************************************
	*                  BUFFER
	*****************************************************************************/

static I2C::Task buffer[I2C_BUFFER_SIZE];

static uint8_t addrCache 	= 0x00;
volatile static bool busy	= false;

volatile static I2C::Id head 	= 1;
volatile static I2C::Id index = 0;

static auto mTask = buffer[index];


#if I2C_BUFFER_SIZE == 255
#define INC(I) ( ++I )
#else
#define INC_inner(I, M, ...) ( I = ( I==(M-1) ) ? 0 : I+1 )
#define INC(...) INC_inner(__VA_ARGS__, I2C_BUFFER_SIZE)
#endif


static bool incrWrite(I2C::Task* task) {
	task->buffs++;
	task->nWrite--;

	return task->nWrite;
}

static bool incrRead(I2C::Task* task) {
	task->buffs++;
	task->nRead--;

	return task->nRead;
}

/*****************************************************************************/


/******************************************************************************
 *                  IMPLEMENTATIONS
 *****************************************************************************/

void I2C::initBus(void) {
	I2C::initBus(I2C::I2C_DEFAULT_BAUDRATE);
}

void I2C::initBus(I2C::Rate baudRate) {
	TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;		// Force to IDLE state on reset.

#if I2C_EXTERNAL_PULLUP == false
	pinMode(PIN_WIRE_SDA, INPUT_PULLUP);
	pinMode(PIN_WIRE_SCL, INPUT_PULLUP);
#endif

	// Set BAUD rate 
	I2C::setBusRate(baudRate);

	TWI0.CTRLA = TWI_SDAHOLD_500NS_gc;		// SMBus conform.
	// Clear Master state, MADDR and MDATA.
	TWI0.MCTRLB = TWI_FLUSH_bm;
	// Read interrupt, Write interrupt, TimeOut 200us, Smart mode.
	// TWI0_MCTRLA = TWI_RIEN_bm | TWI_WIEN_bm | TWI_TIMEOUT_200US_gc | TWI_SMEN_bm;
	// Read interrupt, Write interrupt, TimeOut 200us, Smart mode, Enable.
	TWI0.MCTRLA = TWI_RIEN_bm | TWI_WIEN_bm | TWI_TIMEOUT_200US_gc | TWI_SMEN_bm | TWI_ENABLE_bm;
}


void I2C::setBusRate(I2C::Rate speed) {
	uint32_t bitrate;
	uint16_t rise;

	// The values are taken from the wire library
	switch (speed) {
		case I2C::SLOW:
			bitrate	= 100000;
			rise 		= 1000;
			break;

		case I2C::NORMAL:
			bitrate = 400000;
			rise 		= 300;
			break;

		case I2C::FAST:
			bitrate	= 1000000;
			rise 		= 120;
			break;
	}

	// Equation taken from the wire library
	auto baud = (F_CPU_CORRECTED / bitrate - F_CPU_CORRECTED / 1000000 * rise / 1000 - 10) / 2;
	TWI0.MBAUD = (uint8_t)baud;
}


I2C::Id I2C::addTask(I2C::Task newTask) {
	// Wait if the buffer is full
	while (head == index);

	auto id = head;
	buffer[head] = newTask;
	INC(head);

	if ( !busy ) {
		INC(index);
		mTask = buffer[index];
		
		busy = true;
		masterHandler();
	}

	return id;
}

void I2C::cancelTask(I2C::Id taskId) {
	auto& task = buffer[taskId];

	task.nRead = 0;
	task.nWrite = 0;
}

/*****************************************************************************/


/******************************************************************************
 *                  LOGGING
 *****************************************************************************/

#if LOG_INTERRUPTS

namespace ilog {
	enum Event: uint8_t {
		LOG_ERROR_SEND_ADDR,
		LOG_SEND_ADDR,
		LOG_ERROR_WRITE_DATA,
		LOG_WRITE_DATA,
		LOG_ERROR_WRITE_REG,
		LOG_WRITE_REG,
		LOG_ERROR_READ_DATA,
		LOG_READ_DATA,
		LOG_ERROR_READ_LAST,
		LOG_READ_LAST,

		LOG_AWAITING_DATA,
		LOG_AWAITING_LAST,

		LOG_DONE,
		LOG_DONE_WRITE,
		LOG_DONE_READ,

		LOG_NEXT,
		LOG_NEXT_TASK,

		LOG_ACKING,
		LOG_NACKING,

		LOG_RETRYING
	};


	static Event buffer[32] = {};

	volatile uint8_t head		= 1;
	volatile uint8_t index	= 0;

	volatile bool	full			= false;


	static void add(Event e) {
		if ( full || (full = head == index) ) return;

		INC( head, sizeof(buffer) );
		buffer[head] = e;
	}
}


#define ADD_LOG(E) (ilog::add(E))

void I2C::outputInterruptLogs() {
	if (ilog::full) Log.errorln("The log buffer was full");

	uint8_t last = head;
	for ( auto& i = ilog::index; i <= last; INC(i, sizeof(ilog::buffer)) ) switch (ilog::buffer[i]) {

		case ilog::LOG_ERROR_SEND_ADDR:
			Log.warningln("1Failed to send device addresse");
			break;

		case ilog::LOG_SEND_ADDR:
			Log.traceln("Sent device addresse");
			break;

		case ilog::LOG_ERROR_WRITE_DATA:
			Log.warningln("2Failed to write data to bus");
			break;

		case ilog::LOG_WRITE_DATA:
			Log.traceln("Wrote data to the bus");
			break;

		case ilog::LOG_ERROR_WRITE_REG:
			Log.warningln("3Failed to write target register");
			break;

		case ilog::LOG_WRITE_REG:
			Log.traceln("Wrote target register");
			break;

		case ilog::LOG_ERROR_READ_DATA:
			Log.warningln("4Failed to read data from the bus");
			break;
		
		case ilog::LOG_READ_DATA:
			Log.traceln("Read data from bus");
			break;

		case ilog::LOG_ERROR_READ_LAST:
			Log.warningln("5Failed to read last data from the bus");
			break;

		case ilog::LOG_READ_LAST:
			Log.traceln("Read last data from bus");
			break;

		case ilog::LOG_AWAITING_DATA:
			Log.traceln("Awaiting data");
			break;

		case ilog::LOG_AWAITING_LAST:
			Log.traceln("Awaiting the final data");
			break;

		case ilog::LOG_DONE:
			Log.traceln("All data have been transmited");
			break;

		case ilog::LOG_DONE_WRITE:
			Log.traceln("All data have been written");
			break;

		case ilog::LOG_DONE_READ:
			Log.traceln("All data have been read");
			break;

		case ilog::LOG_NEXT:
			break;

		case ilog::LOG_NEXT_TASK:
			Log.traceln("Moving on to next task");

		case ilog::LOG_ACKING:
			Log.traceln("Will acknowlledge next data");
			break;

		case ilog::LOG_NACKING:
			Log.traceln("Will not acknowledge next data");
			break;

		case ilog::LOG_RETRYING:
			break;
	}

	ilog::full = false;
}

#else 
#define ADD_LOG(E) ( (void)0 )
#endif

/*****************************************************************************/


/******************************************************************************
 *                  INTERRUPTS
 *****************************************************************************/

static void sendAddr(I2C::Task& task) {
	if ( TWI0.MSTATUS & (TWI_ARBLOST_bm | TWI_BUSERR_bm) ) {
		ADD_LOG(ilog::LOG_ERROR_SEND_ADDR);
		return;
	}

	ADD_LOG(ilog::LOG_SEND_ADDR);

	TWI0.MADDR = *task.buffs;
	incrWrite(&task);

	if ( task.nWrite == 1 ) {
		ADD_LOG(ilog::LOG_NACKING);
		task.action	= I2C::LAST_WRITE;
	} else {
		ADD_LOG(ilog::LOG_ACKING);
		task.action = I2C::WRITE_DATA;
	}
}

static void sendReadAddr(I2C::Task& task, uint8_t addr) {
	if ( TWI0.MSTATUS & (TWI_ARBLOST_bm | TWI_BUSERR_bm) ) {
		ADD_LOG(ilog::LOG_ERROR_SEND_ADDR);
		return;
	}

	ADD_LOG(ilog::LOG_SEND_ADDR);
	TWI0.MADDR = addr | 0x01;

	if ( task.nRead == 1 ) {
		ADD_LOG(ilog::LOG_NACKING);
		task.action	= I2C::LAST_READ;
	} else {
		ADD_LOG(ilog::LOG_ACKING);
		task.action = I2C::READ_DATA;
	}
}


static void stop(I2C::Task& task) {
	task.buffs[1] = I2C::OK;

	while (task.nWrite == 0) {
		INC(index);

		// Nothing else to transmit
		if ( index == head ) {
			ADD_LOG(ilog::LOG_DONE);

			TWI0.MCTRLB |= TWI_MCMD_STOP_gc;
			busy = false;
			return;
		}
	}

	ADD_LOG(ilog::LOG_NEXT_TASK);

	mTask = buffer[index];
	sendAddr(mTask);
}

static void reset(I2C::Task& task) {
	ADD_LOG(ilog::LOG_RETRYING);

	mTask = buffer[index];
	sendAddr(mTask);
}

static void handleError(uint8_t status) {
	
}


static void writeData(I2C::Task& task) {
	if ( TWI0.MSTATUS & (TWI_ARBLOST_bm | TWI_BUSERR_bm | TWI_RXACK_bm) ) {
		ADD_LOG(ilog::LOG_ERROR_WRITE_DATA);
		return;
	}

	ADD_LOG(ilog::LOG_WRITE_DATA);
	TWI0.MDATA = *task.buffs;
	
	if ( incrWrite(&task) == 1 ) {
		ADD_LOG(ilog::LOG_DONE_WRITE);
		task.action = I2C::LAST_WRITE;
	}
}

static void writeLast(I2C::Task& task) {
	if ( TWI0.MSTATUS & (TWI_ARBLOST_bm | TWI_BUSERR_bm | TWI_RXACK_bm) ) {
		ADD_LOG(ilog::LOG_ERROR_WRITE_REG);
		return;
	}
	ADD_LOG(ilog::LOG_WRITE_REG);

	TWI0.MDATA = *task.buffs;
	incrWrite(&task);

	task.action = (task.nRead == 0) ? I2C::STOP : I2C::SEND_ADDR_READ;
}

static void readData(I2C::Task& task) {
	if ( (TWI0.MSTATUS ^ TWI_RIF_bm) & (TWI_ARBLOST_bm | TWI_BUSERR_bm | TWI_RXACK_bm | TWI_RIF_bm) ) {
		ADD_LOG(ilog::LOG_ERROR_READ_DATA);
		return;
	}

	ADD_LOG(ilog::LOG_READ_DATA);

	TWI0.MCTRLB |= TWI_ACKACT_ACK_gc;
	*task.buffs	= TWI0.MDATA;
	
	if ( incrRead(&task) == 1 ) {
		ADD_LOG(ilog::LOG_NACKING);
		task.action	= I2C::LAST_READ;
	} else {
		TWI0.MCTRLB |= TWI_MCMD_RECVTRANS_gc;
	}
}

static void readLast(I2C::Task& task) {
	if ( (TWI0.MSTATUS ^ TWI_RIF_bm) & (TWI_ARBLOST_bm | TWI_BUSERR_bm | TWI_RXACK_bm | TWI_RIF_bm) ) {
		ADD_LOG(ilog::LOG_ERROR_READ_LAST);
		return;
	}

	ADD_LOG(ilog::LOG_READ_LAST);

	TWI0.MCTRLB |= TWI_ACKACT_NACK_gc;
	*task.buffs = TWI0.MDATA;

	incrRead(&task);
	stop(task);
}


static void masterHandler(void) {
	switch (mTask.action) {
		case I2C::SEND_ADDR:
			sendAddr(mTask);
			break;

		case I2C::SEND_ADDR_READ:
			sendReadAddr(mTask, *buffer[index].buffs);
			break;

		case I2C::WRITE_DATA:
			writeData(mTask);
			break;

		case I2C::LAST_WRITE:
			writeLast(mTask);
			break;

		case I2C::READ_DATA:
			readData(mTask);
			break;

		case I2C::LAST_READ:
			readLast(mTask);
			break;

		case I2C::STOP:
			stop(mTask);
			break;


		case I2C::AWAIT_READ:
			ADD_LOG(ilog::LOG_AWAITING_DATA);
			mTask.action = I2C::READ_DATA;
			break;

		case I2C::AWAIT_LAST_READ:
			ADD_LOG(ilog::LOG_AWAITING_DATA);
			mTask.action = I2C::LAST_READ;
			break;
	}
}


ISR(TWI0_TWIM_vect) {
	masterHandler();
}


ISR(TWI0_TWIS_vect) {

}

/*****************************************************************************/
