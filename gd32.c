/******************************************************************************
 *
 *  Yaskawa SGDXS Servo Motor Control via IgH EtherCAT Master
 *  Based on sigma-X servo drive
 *
 *****************************************************************************/
#define _GNU_SOURCE

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>
#include <sys/mman.h>
#include <malloc.h>
#include <stdint.h>
#include <sched.h>

#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <stdarg.h>

#include "ecrt.h"

#define USE_PRINT 1

#define STW_RESET_MASK 0x4f //0x40
#define STW_READY_MASK 0x6f //0x61
#define STW_SWITCH_ON_MASK 0x027f //0x0233
#define STW_OPER_MASK 0x027f //0x0237

#define STW_RESET_VAL 0x40
#define STW_READY_VAL 0x61
#define STW_SWITCH_ON_VAL 0x0233
#define STW_OPER_VAL 0x0237
/****************************************************************************/
static inline void ts_printf(const char *fmt, ...)
{
#if USE_PRINT
    struct timeval tv;
    struct tm tm_time;

    gettimeofday(&tv, NULL);
    localtime_r(&tv.tv_sec, &tm_time);

    printf("[%04d-%02d-%02d %02d:%02d:%02d.%06ld] ",
           tm_time.tm_year + 1900,
           tm_time.tm_mon + 1,
           tm_time.tm_mday,
           tm_time.tm_hour,
           tm_time.tm_min,
           tm_time.tm_sec,
           tv.tv_usec);

    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
#else
    return;
#endif
}

#define printf(...) ts_printf(__VA_ARGS__)
/****************************************************************************/
// Define NSEC_PER_SEC if not available
#ifndef NSEC_PER_SEC
#define NSEC_PER_SEC 1000000000L
#endif

/****************************************************************************/

// Application parameters
#define FREQUENCY 1000  // Task frequency in Hz
#define CLOCK_TO_USE CLOCK_MONOTONIC
#define MEASURE_TIMING

/****************************************************************************/

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static ec_slave_config_t *sc_servo = NULL;
static ec_slave_config_state_t sc_servo_state = {};

/****************************************************************************/

// Process data
static uint8_t *domain1_pd = NULL;

#define ServoPos 0, 0                         // Alias and position on the bus
#define Yaskawa_SGDXS 0x00000f82, 0x00000001  // Vendor ID, Product code

// Offsets for PDO entries
static unsigned int off_servo_controlword;
static unsigned int off_servo_target_position;
static unsigned int off_servo_target_velocity;
static unsigned int off_servo_modes_of_operation;

static unsigned int off_servo_statusword;
static unsigned int off_servo_position_actual;
static unsigned int off_servo_velocity_actual;
static unsigned int off_servo_modes_of_operation_display;

static unsigned int off_servo_gap_val;

/****************************************************************************/

// PDO entries - RxPDO (outputs from master to slave)  18 byte
static ec_pdo_entry_info_t servo_pdo_entries_rx[] = {
        {0x6040, 0x00, 16},  // Controlword
        {0x607a, 0x00, 32},  // Target position
        {0x60ff, 0x00, 32},  // Target velocity
        {0x6060, 0x00, 8},   // Modes of operation
        {0x0000, 0x00, 8},   // Gap
};

// PDO entries - TxPDO (inputs from slave to master) 20 byte
static ec_pdo_entry_info_t servo_pdo_entries_tx[] = {
        {0x6041, 0x00, 16},  // Statusword
        {0x6064, 0x00, 32},  // Position actual value
        {0x606c, 0x00, 32},  // Velocity actual value
        {0x6061, 0x00, 8},   // Modes of operation display
        {0x0000, 0x00, 8},   // Gap
};

// PDO configuration
static ec_pdo_info_t servo_pdos[] = {
        {0x1600, sizeof(servo_pdo_entries_rx)/sizeof(ec_pdo_entry_info_t), servo_pdo_entries_rx},  // RxPDO
        {0x1a00, sizeof(servo_pdo_entries_tx)/sizeof(ec_pdo_entry_info_t), servo_pdo_entries_tx},  // TxPDO
};

// Sync manager configuration
static ec_sync_info_t servo_syncs[] = {
        // {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
        // {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
        {2, EC_DIR_OUTPUT, 1, servo_pdos + 0},
        {3, EC_DIR_INPUT, 1, servo_pdos + 1},
        {0xff}};
/****************************************************************************/

// Motion control variables
static int32_t target_position = 0;
static int32_t distance = 999;
static struct timespec next_switch_time;
static int position_toggle = 0;

// Statistics
static unsigned int counter = 0;

/****************************************************************************/

struct timespec timespec_add(struct timespec time1, struct timespec time2)
{
    struct timespec result;

    if ((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC) {
        result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
    }
    else {
        result.tv_sec = time1.tv_sec + time2.tv_sec;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
    }

    return result;
}

/****************************************************************************/

void check_domain1_state(void)
{
    ec_domain_state_t ds;

    ecrt_domain_state(domain1, &ds);

    if (ds.working_counter != domain1_state.working_counter) {
        // printf("Domain1: WC %u.\n", ds.working_counter);
    }
    if (ds.wc_state != domain1_state.wc_state) {
        // printf("Domain1: State %u.\n", ds.wc_state);
    }

    domain1_state = ds;
}

/****************************************************************************/

void check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding) {
        printf("%u slave(s).\n", ms.slaves_responding);
    }
    if (ms.al_states != master_state.al_states) {
        printf("AL states: 0x%02X.\n", ms.al_states);
    }
    if (ms.link_up != master_state.link_up) {
        printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }

    master_state = ms;
}

/****************************************************************************/

void check_slave_config_states(void)
{
    ec_slave_config_state_t s;

    ecrt_slave_config_state(sc_servo, &s);

    if (s.al_state != sc_servo_state.al_state) {
        printf("Servo: AL State 0x%02X.\n", s.al_state);
    }
    if (s.online != sc_servo_state.online) {
        printf("Servo: %s.\n", s.online ? "online" : "offline");
    }
    if (s.operational != sc_servo_state.operational) {
        printf("Servo: %soperational.\n", s.operational ? "" : "Not ");
    }

    sc_servo_state = s;
}

uint32_t get_servo_al_state(void)
{
    ec_slave_config_state_t s;

    ecrt_slave_config_state(sc_servo, &s);
    return s.al_state;
}

uint16_t get_servo_status_word(void)
{
    uint16_t stw = EC_READ_U16(domain1_pd + off_servo_statusword);
    return stw;
}


/****************************************************************************/

int configure_servo_sdo(void)
{
    uint8_t u8_val;
    uint16_t u16_val;
    int16_t i16_val;
    uint32_t u32_val;
    int32_t i32_val;

    printf("Configuring servo SDOs...\n");

    u16_val = 0x0;
    if (ecrt_slave_config_sdo16(sc_servo, 0x6040, 0x00, u16_val) != 0) {
        fprintf(stderr, "Failed to reset controlword\n");
        return -1;
    }
    usleep(1000);

    // Set Profile Position mode
    u8_val = 1;
    if (ecrt_slave_config_sdo8(sc_servo, 0x6060, 0x00, u8_val) != 0) {
        fprintf(stderr, "Failed to set PP mode\n");
        return -1;
    }
    usleep(1000);

    // config PDO mapping ------------------------------------------- start
    // ecrt_slave_config_sdo8(sc_servo, 0x1A00, 0, 0);
    // ecrt_slave_config_sdo32(sc_servo, 0x1A00, 1, 0x60410010);//stw
    // ecrt_slave_config_sdo32(sc_servo, 0x1A00, 2, 0x60640020);//pos act
    // ecrt_slave_config_sdo32(sc_servo, 0x1A00, 3, 0x606c0020);//vel act
    // ecrt_slave_config_sdo32(sc_servo, 0x1A00, 4, 0x60610008);//mod disp
    // ecrt_slave_config_sdo32(sc_servo, 0x1A00, 5, 0x00000008);//Gap
    // ecrt_slave_config_sdo8(sc_servo, 0x1A00, 0, 5);

    // ecrt_slave_config_sdo8(sc_servo, 0x1600, 0, 0);
    // ecrt_slave_config_sdo32(sc_servo, 0x1600, 1, 0x60400010);//ctl
    // ecrt_slave_config_sdo32(sc_servo, 0x1600, 2, 0x607a0020);//pos targ
    // ecrt_slave_config_sdo32(sc_servo, 0x1600, 3, 0x60ff0020);//vel targ
    // ecrt_slave_config_sdo32(sc_servo, 0x1600, 4, 0x60600008);//mod
    // ecrt_slave_config_sdo32(sc_servo, 0x1600, 5, 0x00000008);//Gap
    // ecrt_slave_config_sdo8(sc_servo, 0x1600, 0, 5);

    ecrt_slave_config_sdo8(sc_servo, 0x1c12, 0, 0);
    usleep(1000);
    ecrt_slave_config_sdo16(sc_servo, 0x1c12, 1, 0x1600);
    usleep(1000);
    ecrt_slave_config_sdo8(sc_servo, 0x1c12, 0, 1);
    usleep(1000);

    ecrt_slave_config_sdo8(sc_servo, 0x1c13, 0, 0);
    usleep(1000);
    ecrt_slave_config_sdo16(sc_servo, 0x1c13, 1, 0x1a00);
    usleep(1000);
    ecrt_slave_config_sdo8(sc_servo, 0x1c13, 0, 1);
    usleep(1000);

    printf("SDO configuration completed successfully.\n");
    return 0;
}

/****************************************************************************/

void cyclic_task()
{
    struct timespec current_time;
    static int32_t last_target_position = 0;

    static struct timespec print_time;

    // Receive process data
    ecrt_master_receive(master);
    ecrt_domain_process(domain1);

    check_domain1_state();

    // Read inputs
    uint16_t statusword = EC_READ_U16(domain1_pd + off_servo_statusword);
    int32_t position_actual = EC_READ_S32(domain1_pd + off_servo_position_actual);

    // Toggle target position every 3 seconds
    clock_gettime(CLOCK_TO_USE, &current_time);
    if (current_time.tv_sec > next_switch_time.tv_sec
        || (current_time.tv_sec == next_switch_time.tv_sec
            && current_time.tv_nsec >= next_switch_time.tv_nsec)) {
        // Toggle position
        if (position_toggle) {
            target_position = 0xaabbccdd;//0;
            position_toggle = 0;
        }
        else {
            target_position = 0xaabbccdd;//distance;
            position_toggle = 1;
        }

        // Set next switch time
        next_switch_time.tv_sec += 5;

        printf("Switching target position to: %08X\n", (uint32_t)target_position);
    }

    // Write outputs
    uint16_t controlword;
    if (target_position != last_target_position) {
        last_target_position = target_position;
        controlword = 0x000F;  // 0x1f, New position, set bit 4
        EC_WRITE_U16(domain1_pd + off_servo_controlword, controlword);
        EC_WRITE_S32(domain1_pd + off_servo_target_position, target_position);
        printf("ctrlWord 0x1F send\n");
    }
    else {
        controlword = 0x000F;  // Normal operation
        EC_WRITE_U16(domain1_pd + off_servo_controlword, controlword);
    }

    // Send process data
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);

    // Display status every second
    clock_gettime(CLOCK_TO_USE, &current_time);
    if ((current_time.tv_sec - print_time.tv_sec)*1000000000 + (current_time.tv_nsec - print_time.tv_nsec)
         > 1000000000) {

        print_time.tv_sec = current_time.tv_sec + 1;
        print_time.tv_nsec = current_time.tv_nsec;

        printf("Status: 0x%04X | Pos: 0x%08X/0x%08X | AL status: %d\n",
        statusword,
        position_actual,
        target_position,
        get_servo_al_state());
    }
}

/****************************************************************************/

void signal_handler(int sig)
{
    printf("\nReceived signal %d, shutting down...\n", sig);

    // Zero outputs
    if (domain1_pd) {
        EC_WRITE_U16(domain1_pd + off_servo_controlword, 0);//cia402 disable voltage, goto init state
        EC_WRITE_S32(domain1_pd + off_servo_target_position, 0);
        EC_WRITE_S32(domain1_pd + off_servo_target_velocity, 0);
        EC_WRITE_U8(domain1_pd + off_servo_modes_of_operation, 0);

        ecrt_domain_queue(domain1);
        ecrt_master_send(master);
    }

    if (master) {
        ecrt_release_master(master);
    }

    exit(0);
}

/****************************************************************************/
#include <sched.h>
#include <stdio.h>
#include <errno.h>
int main(int argc, char **argv)
{
    struct timespec wakeup_time, current_time;
    struct timespec interval;
    int ret;

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        perror("mlockall failed");
        return -1;
    }

    printf("Starting Yaskawa SGDXS EtherCAT control...\n");

    // Set up signal handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Request master
    master = ecrt_request_master(0);
    if (!master) {
        fprintf(stderr, "Failed to request master.\n");
        return -1;
    }

    // Create domain
    domain1 = ecrt_master_create_domain(master);
    if (!domain1) {
        fprintf(stderr, "Failed to create domain.\n");
        goto out_release_master;
    }

    // Configure servo slave
    if (!(sc_servo = ecrt_master_slave_config(
                  master,
                  ServoPos,
                  Yaskawa_SGDXS))) {
        fprintf(stderr, "Failed to get slave configuration.\n");
        goto out_release_master;
    }

    printf("check AL state after reset...\n");
    check_slave_config_states();

    // Configure SDOs
    if (configure_servo_sdo() != 0) {
        fprintf(stderr, "Failed to configure SDOs.\n");
        goto out_release_master;
    }

    // Configure PDOs
    printf("Configuring PDOs...\n");
    if (ecrt_slave_config_pdos(sc_servo, EC_END, servo_syncs)) {
        fprintf(stderr, "Failed to configure PDOs.\n");
        goto out_release_master;
    }

    // Register PDO entries
    usleep(1000);
    if (ecrt_domain_reg_pdo_entry_list(domain1,(ec_pdo_entry_reg_t[]) {
        //output
        {ServoPos, Yaskawa_SGDXS, 0x6040, 0x00, &off_servo_controlword},
        {ServoPos, Yaskawa_SGDXS, 0x607a, 0x00, &off_servo_target_position},
        {ServoPos, Yaskawa_SGDXS, 0x60ff, 0x00, &off_servo_target_velocity},
        {ServoPos, Yaskawa_SGDXS, 0x6060, 0x00, &off_servo_modes_of_operation},
        //input
        {ServoPos, Yaskawa_SGDXS, 0x6041, 0x00, &off_servo_statusword},
        {ServoPos, Yaskawa_SGDXS, 0x6064, 0x00, &off_servo_position_actual},
        {ServoPos, Yaskawa_SGDXS, 0x606c, 0x00, &off_servo_velocity_actual},
        {ServoPos, Yaskawa_SGDXS, 0x6061, 0x00, &off_servo_modes_of_operation_display},
        {ServoPos, Yaskawa_SGDXS, 0x0000, 0x00, &off_servo_gap_val},
        {}
    })) {
        fprintf(stderr, "Failed to register PDO entries.\n");
        goto out_release_master;
    }

    // ecrt_slave_config_dc(sc_servo, 	0x0300, 4000000, 125000, 0, 0);

    printf("Activating master...\n");
    if (ecrt_master_activate(master)) {
        fprintf(stderr, "Failed to activate master.\n");
        goto out_release_master;
    }

    printf("check AL state after Activate...\n");
    check_slave_config_states();

    if (!(domain1_pd = ecrt_domain_data(domain1))) {
        fprintf(stderr, "Failed to get domain data pointer.\n");
        goto out_release_master;
    }

    EC_WRITE_U16(domain1_pd + off_servo_controlword, 0);
    EC_WRITE_S32(domain1_pd + off_servo_target_position, 0);
    EC_WRITE_S32(domain1_pd + off_servo_target_velocity, 0);
    EC_WRITE_U8(domain1_pd + off_servo_modes_of_operation, 1);  // mode

    ecrt_domain_queue(domain1);
    ecrt_master_send(master);

    // Set realtime priority
    struct sched_param param = {};
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);

    printf("Using priority %i.\n", param.sched_priority);
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("sched_setscheduler failed");
    }

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(3, &cpuset);  // 绑定到 CPU3（假设这是隔离核）
    if (sched_setaffinity(0, sizeof(cpuset), &cpuset) == -1) {
        perror("sched_setaffinity failed");
    }

    printf("Starting cyclic task at %d Hz...\n", FREQUENCY);

    // Initialize timing
    clock_gettime(CLOCK_TO_USE, &wakeup_time);
    clock_gettime(CLOCK_TO_USE, &next_switch_time);
    next_switch_time.tv_sec += 3;  // First switch after 3 seconds

    interval.tv_sec = 0;
    interval.tv_nsec = 1000000000 / FREQUENCY;

    while(get_servo_al_state() != 0x08)
    {
		ecrt_master_receive(master);
		ecrt_domain_process(domain1);
		
        ecrt_domain_queue(domain1);
        ecrt_master_send(master);
        usleep(1000);
    }
    printf("enter OP state OK\n");


    usleep(1000);//maybe need adjustment
    EC_WRITE_U16(domain1_pd + off_servo_controlword, 6);
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);

    usleep(1000);
    EC_WRITE_U16(domain1_pd + off_servo_controlword, 7);
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);

    usleep(1000);
    EC_WRITE_U16(domain1_pd + off_servo_controlword, 15);
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);
    usleep(1000);

    // printf("CiA402 status check start...\n");
    // while(get_servo_status_word() & STW_OPER_MASK != STW_OPER_VAL)
    // {
	// 	ecrt_master_receive(master);
	// 	ecrt_domain_process(domain1);
		
    //     ecrt_domain_queue(domain1);
    //     ecrt_master_send(master);
    //     usleep(1000);
    // }
    // printf("CiA402 status check OK\n");
    // usleep(1000);

    // Cyclic task loop
    while (1) {
        wakeup_time = timespec_add(wakeup_time, interval);
        clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeup_time, NULL);

        // wakeup_time.tv_sec = 0;
        // wakeup_time.tv_nsec = 10000;//5us
        // clock_nanosleep(CLOCK_TO_USE, 0, &wakeup_time, NULL);

        cyclic_task();
    }

    return 0;

out_release_master:
    printf("Releasing master...\n");
    ecrt_release_master(master);
    return -1;
}

/****************************************************************************/
