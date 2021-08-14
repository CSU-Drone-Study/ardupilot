#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/RingBuffer.h>

#if HAL_OS_POSIX_IO
#include <stdio.h>
#endif

void setup();
void loop();
static void setup_uart(AP_HAL::UARTDriver *uart);
static void printAllAvailableBytes();

//HAL variables
const AP_HAL::HAL& hal = AP_HAL::get_HAL();
static AP_HAL::UARTDriver *imu = hal.serial(4);
static AP_HAL::UARTDriver *console = hal.serial(0);

//shared ring buffer
static const uint32_t bufferSize = 1024;
static ByteBuffer buffer{bufferSize};
static uint8_t tempData[bufferSize];    //later, the producer and consumer should each have their own temp buffers

void setup() {
    setup_uart(imu);
    setup_uart(console);
    hal.scheduler->delay(1000); //Ensure that the uart can be initialized
}

void loop() {

    printAllAvailableBytes();
    console -> print("bruh");
    hal.scheduler -> delay(10);
}

//just prints whatever is in the ring buffer
static void printAllAvailableBytes() {
    uint32_t available = imu -> read(tempData, bufferSize);
    for(int i = 0; i < available; i++) {
        console -> printf("0x%x ", tempData[i]);
    }
    console -> printf("\n");
}

static void setup_uart(AP_HAL::UARTDriver *uart) {
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }
    //uart->set_stop_bits(1);
    //uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    uart->begin(115200);
}

AP_HAL_MAIN();