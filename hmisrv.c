#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
#include <linux/input.h>
#include <signal.h>
#include <pthread.h>
#include <sys/time.h>
#include <stdbool.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <sys/sysinfo.h>
#include <netdb.h>  /* For getnameinfo, NI_MAXHOST, NI_NUMERICHOST */
#include <sys/wait.h>
#include <time.h>
#include <libudev.h>
#include <poll.h>
#include <sys/stat.h>
/* -----------------------------------------------------------------------------
 * Configuration definitions
 * -----------------------------------------------------------------------------*/

// Default Device paths
#define DEFAULT_INPUT_DEVICE "/dev/input/event0"
#define DEFAULT_SERIAL_DEVICE "/dev/ttyACM0"
// For Automatic detection of hid-display-dongle
#define HMI_VENDOR_ID "1209"
#define HMI_PRODUCT_ID "0001"
#define HMI_PRODUCT_NAME "Pico Encoder Display"
#define HMI_MANUFACTURER "DIY Projects"
#define DETECTION_POLL_INTERVAL 2000  // Poll every 2 seconds
#define UDEV_POLL_TIMEOUT 100         // 100ms timeout for udev events

// Protocol commands
#define CMD_CLEAR 0x01
#define CMD_DRAW_TEXT 0x02
#define CMD_SET_CURSOR 0x03
#define CMD_INVERT 0x04
#define CMD_BRIGHTNESS 0x05
#define CMD_PROGRESS_BAR 0x06

// Display properties
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64
#define CHAR_WIDTH 6
#define CHAR_HEIGHT 8

// Menu configuration
#define MENU_START_Y 16
#define MENU_ITEM_SPACING 8 //10
#define MENU_MAX_ITEMS 10
#define MENU_TITLE "MAIN MENU"
#define MENU_SEPARATOR "----------------"
#define MENU_VISIBLE_ITEMS 6    // Maximum number of menu items visible at once
#define MENU_SCROLL_INDICATOR_WIDTH 3  // Width of scroll indicator in pixels
#define STAT_UPDATE_SEC 1  // Update system stats every 1 second

// Input event codes (already defined in linux/input.h but replicated here for clarity)
// #define EV_REL 0x02  // Relative movement event type
// #define EV_KEY 0x01  // Key event type
// #define REL_X  0x00  // X-axis code
// #define BTN_LEFT 0x110  // Left mouse button code

// Timing constants
#define DISPLAY_CMD_DELAY 10000  // 10ms delay between display commands
#define DISPLAY_CLEAR_DELAY 50000 // 50ms delay after clear
#define DISPLAY_UPDATE_DEBOUNCE 100 // 100ms between display updates
#define KEY_ACCELERATION_THRESHOLD 200 // 200ms for acceleration
#define EVENT_PROCESS_THRESHOLD 30 // 30ms for event processing
#define INPUT_SELECT_TIMEOUT 20000 // 20ms timeout for select
#define MAIN_LOOP_DELAY 5000 // 5ms delay in main loop
#define STARTUP_DELAY 1000000 // 1s delay at startup
#define CMD_BUFFER_FLUSH_INTERVAL 50 // 50ms between buffer flushes

// power save constants
#define POWER_SAVE_TIMEOUT_SEC 10  // Default timeout in seconds before entering power save mode
#define CMD_POWER_MODE 0x07        // Command byte for power mode control

// Input event handling limits
#define MAX_EVENTS_PER_ITERATION 5
#define MAX_ACCELERATION_STEPS 3

// Serial command buffer
#define CMD_BUFFER_SIZE 256

// Program version
#define VERSION "1.2.0"
#define MAX_MENU_ITEMS 10  // Maximum number of menu items we can handle

/* -----------------------------------------------------------------------------
 * Data structures
 * -----------------------------------------------------------------------------*/

// Menu item structure
typedef struct {
    const char *label;
    void (*action)(void);
} MenuItem;

// SerialCommand buffer structure
typedef struct {
    unsigned char buffer[CMD_BUFFER_SIZE];
    size_t used;
    struct timeval last_flush;
} SerialCmdBuffer;

// Display state structure
typedef struct {
    int inverted;
    int current_menu_item;
    int brightness;
    struct timeval last_update_time;
    int update_in_progress;
    int needs_update;
    int menu_scroll_offset;    // First visible menu item index
} DisplayState;

// Input state structure
typedef struct {
    struct timeval last_key_time;
    int consecutive_same_key;
    int last_direction;
    struct timeval last_event_time;
    int paired_event_count;
    int total_rel_x;
} InputState;

// Configuration structure
typedef struct {
    const char *input_device;
    const char *serial_device;
    bool verbose_mode;
} Config;

//Define menu item configuration flags
typedef struct {
    const char *label;         // Menu item label
    void (*action)(void);      // Function to call when selected
    bool enabled;              // Whether this item is enabled
} MenuItemConfig;

/* -----------------------------------------------------------------------------
 * Global variables
 * -----------------------------------------------------------------------------*/

// File descriptors
static int input_fd = -1;
static int serial_fd = -1;

// Locks and flags
static int running = 1;
static pthread_mutex_t serial_mutex = PTHREAD_MUTEX_INITIALIZER;

// State structures
static Config config = {
    .input_device = DEFAULT_INPUT_DEVICE,
    .serial_device = DEFAULT_SERIAL_DEVICE,
    .verbose_mode = false
};

static DisplayState display_state = {
    .inverted = 0,
    .current_menu_item = 0,
    .brightness = 128,
    .last_update_time = {0, 0},
    .update_in_progress = 0,
    .needs_update = 0,
    .menu_scroll_offset = 0
};

static InputState input_state = {
    .last_key_time = {0, 0},
    .consecutive_same_key = 0,
    .last_direction = 0,
    .last_event_time = {0, 0},
    .paired_event_count = 0,
    .total_rel_x = 0
};

static SerialCmdBuffer cmd_buffer = {
    .buffer = {0},
    .used = 0,
    .last_flush = {0, 0}
};

// global variables for device detection thread
static pthread_t disconnect_monitor_tid;
static volatile bool device_disconnected = false;
static volatile bool monitor_thread_running = false;

// Menu data
static int menu_item_count = 0;
static MenuItem *menu_items = NULL;

// display power save variables
static bool power_save_enabled = false;      // Whether power save mode is enabled
static bool display_powered_on = true;       // Current display power state
static struct timeval last_activity_time;    // Time of last user activity
static void (*current_submenu_function)(void) = NULL;
static volatile bool exit_current_submenu = false;
static volatile bool power_save_activated = false;

/* -----------------------------------------------------------------------------
 * Helper macros and function prototypes
 * -----------------------------------------------------------------------------*/

// Helper macros for conditional debug output
#define DEBUG_PRINT(fmt, ...) \
    do { if (config.verbose_mode) fprintf(stderr, fmt, ##__VA_ARGS__); } while (0)

// Function prototypes
// Function prototypes for device detection
void detect_and_run(void);
char* find_hmi_input_device(void);
char* find_hmi_serial_device(void);
bool check_device_present(void);
bool check_device_connected(void);
void *disconnection_monitor_thread(void *arg);
void monitor_device_until_connected(void);
bool check_device_present_silent(void);
// Menu functions
void init_menu(void);
void cleanup_menu(void);
void update_menu_display(void);
void update_selection(int old_selection, int new_selection);
void configure_menu_for_minimal();
void configure_menu_for_full();
void set_menu_item_enabled(int index, bool enabled);
void reload_menu(void);

// Input handling
void handle_key_left(void);
void handle_key_right(void);
void handle_key_enter(void);
void handle_input_events(void);

// Display commands
void send_clear(void);
void send_draw_text(int x, int y, const char *text);
void send_set_cursor(int x, int y);
void send_invert(int invert);
void send_command(const unsigned char *data, size_t length);
void buffer_command(const unsigned char *data, size_t length);
void flush_cmd_buffer(void);
void send_brightness(int brightness);
void send_progress_bar(int x, int y, int width, int height, int percentage);

// Brightness control
void action_brightness(void);
void update_brightness_value(int brightness);
void setup_brightness_screen(void);
void brightness_control_loop(void);

// Network and system stats
void action_network_settings(void);
void action_system_stats(void);
void get_network_info(char *ip_str, size_t ip_len, char *mac_str, size_t mac_len,char* ifname,
		size_t if_len);
//void get_system_info(char *cpu_str, size_t cpu_len, char *mem_str, size_t mem_len);
void get_system_info(char *cpu_str, size_t cpu_len, char *mem_total_str, size_t mem_total_len,
                    char *mem_free_str, size_t mem_free_len);//)
void get_system_info_with_percent(char *cpu_str, size_t cpu_len, char *mem_total_str,
                               size_t mem_total_len, char *mem_free_str, size_t mem_free_len,
                               int *cpu_percentage, int *mem_percentage);
void action_test_internet(void);
int ping_server(const char *server, int timeout_sec);
void set_wifi_status(int state);
int get_wifi_status(void);

// Menu actions
void action_hello(void);
void action_counter(void);
void action_invert(void);
void action_exit(void);
void action_wifi_menu(void);

// Utility functions
void print_usage(const char *program_name);
void handle_signal(int sig);
void set_nonblocking(int fd);
int open_input_device(const char *device_path);
int open_serial_device(const char *device_path);
long get_time_diff_ms(struct timeval *start, struct timeval *end);

//power save functions
void set_display_power(bool power_on);
void update_activity_timestamp(void);
void check_power_save_timeout(void);
void check_power_save_and_signal_exit(void);

static MenuItemConfig menu_item_configs[] = {
    {"Hello World",      action_hello,          false},  // 0
    {"Counter",          action_counter,        false},  // 1
    {"Invert Display",   action_invert,         true},  // 2
    {"Brightness",       action_brightness,     true},  // 3
    {"Net Settings",     action_network_settings, true},  // 4
    {"System Stats",     action_system_stats,   true},  // 5
    {"Test Internet",    action_test_internet,  true},  // 6
    {"WiFi Settings",    action_wifi_menu,      true},  // 8
    {"Exit",             action_exit,           true},  // 9
    // Add new menu items here
    {NULL, NULL, false}  // End marker, always keep this
};

/* -----------------------------------------------------------------------------
 * Menu action functions
 * -----------------------------------------------------------------------------*/
//Example usage - add this function to demonstrate enabling/disabling items
void configure_menu_for_minimal() {
    // Example: Configure for minimal mode (only essential items)
    set_menu_item_enabled(0, false);  // Disable Hello World
    set_menu_item_enabled(1, false);  // Disable Counter
    set_menu_item_enabled(3, true);   // Keep Brightness
    set_menu_item_enabled(6, false);  // Disable Test Internet
    // Always keep Exit enabled
    set_menu_item_enabled(7, true);
    // Reload the menu to apply changes
    reload_menu();
}

void configure_menu_for_full() {
    // Example: Enable all menu items
    for (int i = 0; menu_item_configs[i].label != NULL; i++)
        set_menu_item_enabled(i, true);
    // Reload the menu to apply changes
    reload_menu();
}

// Example menu action functions
void action_hello(void) {
    char text[64];

    // Format all text at once to prevent clearing between each line
    snprintf(text, sizeof(text), "Hello, World!\nPressed Enter!");

    // Clear first, then draw all text at once
    send_clear();
    usleep(DISPLAY_CMD_DELAY * 3);  // 30ms delay
    send_draw_text(0, 0, text);

    // Wait for user to see result before returning to menu
    sleep(2);
    update_menu_display();
}

void action_counter(void) {
    static int counter = 0;
    char text[64];

    counter++;

    // Format all text at once to prevent clearing between each line
    snprintf(text, sizeof(text), "Counter:\n%d", counter);

    // Clear first, then draw all text at once
    send_clear();
    usleep(DISPLAY_CMD_DELAY * 3);  // 30ms delay
    send_draw_text(0, 0, text);

    // Wait for user to see result before returning to menu
    sleep(2);
    update_menu_display();
}

void action_invert(void) {
    display_state.inverted = !display_state.inverted;
    send_invert(display_state.inverted);
}

void action_exit(void) {
    running = 0;
}

/* -----------------------------------------------------------------------------
 * Utility functions
 * -----------------------------------------------------------------------------*/

// Calculate time difference in milliseconds
long get_time_diff_ms(struct timeval *start, struct timeval *end) {
    return (end->tv_sec - start->tv_sec) * 1000 +
           (end->tv_usec - start->tv_usec) / 1000;
}

// Print usage instructions
void print_usage(const char *program_name) {
    printf("OLED Menu Control Daemon v%s\n", VERSION);
    printf("Usage: %s [OPTIONS]\n\n", program_name);
    printf("Options:\n");
    printf("  -i DEVICE   Specify input device (default: %s)\n", DEFAULT_INPUT_DEVICE);
    printf("  -s DEVICE   Specify serial device for display (default: %s)\n", DEFAULT_SERIAL_DEVICE);
    printf("  -a          Auto-detect HMI device (overrides -i and -s)\n");
    printf("  -p          Enable power save mode (display turns off after %d seconds of inactivity)\n", POWER_SAVE_TIMEOUT_SEC);
    printf("  -v          Enable verbose debug output\n");
    printf("  -h          Display this help message\n\n");
    printf("Example:\n");
    printf("  %s -i /dev/input/event11 -s /dev/ttyACM0 -v\n\n", program_name);
    printf("Controls:\n");
    printf("  - Rotate encoder left/right to navigate menu\n");
    printf("  - Press encoder button to select menu item\n");
    printf("  - Press Ctrl+C to exit program\n");
}

// Signal handler for clean exit
void handle_signal(int sig) {
    (void)sig;  // Suppress unused parameter warning
    running = 0;
}

// Set non-blocking mode for file descriptor
void set_nonblocking(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

// Open the input device
int open_input_device(const char *device_path) {
    int fd = open(device_path, O_RDONLY);
    if (fd < 0) {
        perror("Failed to open input device");
        return -1;
    }

    // Set non-blocking mode
    set_nonblocking(fd);

    return fd;
}

// Open the serial device
int open_serial_device(const char *device_path) {
    int fd = open(device_path, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        perror("Failed to open serial device");
        return -1;
    }

    // Configure serial port
    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(fd, &tty) != 0) {
        perror("Failed to get serial attributes");
        close(fd);
        return -1;
    }

    // Set baud rate to 115200
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    // 8N1 mode, no flow control
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    // Raw input
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    // Raw output
    tty.c_oflag &= ~OPOST;

    // Set attributes
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("Failed to set serial attributes");
        close(fd);
        return -1;
    }

    // Flush any pending data
    tcflush(fd, TCIOFLUSH);

    return fd;
}

/* -----------------------------------------------------------------------------
 * Serial communication functions
 * -----------------------------------------------------------------------------*/

// Buffer a command to be sent later
void buffer_command(const unsigned char *data, size_t length) {
    pthread_mutex_lock(&serial_mutex);
    // If buffer would overflow, flush it first
    if (cmd_buffer.used + length > CMD_BUFFER_SIZE) {
        flush_cmd_buffer();
    }
    // Copy new command to buffer
    memcpy(cmd_buffer.buffer + cmd_buffer.used, data, length);
    cmd_buffer.used += length;
    // Update last action timestamp
    gettimeofday(&cmd_buffer.last_flush, NULL);
    pthread_mutex_unlock(&serial_mutex);
}

// Flush the command buffer to the serial device
void flush_cmd_buffer(void) {
    pthread_mutex_lock(&serial_mutex);

    if (cmd_buffer.used > 0 && serial_fd >= 0) {
        ssize_t bytes_written = write(serial_fd, cmd_buffer.buffer, cmd_buffer.used);
        if (bytes_written < 0) {
            perror("Error writing to serial device");

            // If error indicates device disconnection, set the flag
            if (errno == EIO || errno == ENODEV || errno == ENXIO) {
                DEBUG_PRINT("Serial buffer write error indicates device disconnection\n");
                device_disconnected = true;
            }
        } else if ((size_t)bytes_written < cmd_buffer.used) {
            fprintf(stderr, "Warning: Only wrote %zd of %zu bytes\n",
                   bytes_written, cmd_buffer.used);
        }

        // Flush the output only if device still connected
        if (!device_disconnected) {
            if (tcdrain(serial_fd) < 0) {
                perror("Error draining serial output");

                // Check if tcdrain error indicates device disconnection
                if (errno == EIO || errno == ENODEV || errno == ENXIO) {
                    DEBUG_PRINT("Serial buffer drain error indicates device disconnection\n");
                    device_disconnected = true;
                }
            }
        }

        cmd_buffer.used = 0;
    }

    pthread_mutex_unlock(&serial_mutex);
}

// Send a command immediately to the serial device
void send_command(const unsigned char *data, size_t length) {
    pthread_mutex_lock(&serial_mutex);
    if (serial_fd >= 0) {
        // Write the data and check return value
        ssize_t bytes_written = write(serial_fd, data, length);
        if (bytes_written < 0) {
            perror("Error writing to serial device");

            // If error indicates device disconnection, set the flag
            if (errno == EIO || errno == ENODEV || errno == ENXIO) {
                DEBUG_PRINT("Serial write error indicates device disconnection\n");
                device_disconnected = true;
            }
        } else if ((size_t)bytes_written < length) {
            fprintf(stderr, "Warning: Only wrote %zd of %zu bytes\n", bytes_written, length);
        }

        // Flush the output buffer to ensure command is sent immediately
        // But only if the device hasn't been disconnected
        if (!device_disconnected) {
            if (tcdrain(serial_fd) < 0) {
                perror("Error draining serial output");

                // Check if tcdrain error indicates device disconnection
                if (errno == EIO || errno == ENODEV || errno == ENXIO) {
                    DEBUG_PRINT("Serial drain error indicates device disconnection\n");
                    device_disconnected = true;
                }
            }
        }
    }
    pthread_mutex_unlock(&serial_mutex);
}

// Clear the display
void send_clear(void) {
    unsigned char cmd = CMD_CLEAR;
    send_command(&cmd, 1);
}

// Draw text at position
void send_draw_text(int x, int y, const char *text) {
    size_t text_len = strlen(text);
    unsigned char *cmd = (unsigned char *)malloc(text_len + 3);
    if (!cmd) {
        fprintf(stderr, "Memory allocation failed in send_draw_text()\n");
        return;
    }

    cmd[0] = CMD_DRAW_TEXT;
    cmd[1] = (unsigned char)x;
    cmd[2] = (unsigned char)y;
    memcpy(cmd + 3, text, text_len);

    send_command(cmd, text_len + 3);
    free(cmd);
}

// Set cursor position
void send_set_cursor(int x, int y) {
    unsigned char cmd[3];
    cmd[0] = CMD_SET_CURSOR;
    cmd[1] = (unsigned char)x;
    cmd[2] = (unsigned char)y;
    send_command(cmd, 3);
}

// Invert display colors
void send_invert(int invert) {
    unsigned char cmd[2];
    cmd[0] = CMD_INVERT;
    cmd[1] = invert ? 1 : 0;
    send_command(cmd, 2);
}

// Send brightness command
void send_brightness(int brightness) {
    unsigned char cmd[2];
    cmd[0] = CMD_BRIGHTNESS;
    cmd[1] = (unsigned char)brightness;
    send_command(cmd, 2);
}

// Send progress bar command
void send_progress_bar(int x, int y, int width, int height, int percentage) {
    unsigned char cmd[6];
    cmd[0] = CMD_PROGRESS_BAR;
    cmd[1] = (unsigned char)x;
    cmd[2] = (unsigned char)y;
    cmd[3] = (unsigned char)width;
    cmd[4] = (unsigned char)height;
    cmd[5] = (unsigned char)percentage;
    send_command(cmd, 6);
}

/* -----------------------------------------------------------------------------
 * Menu management functions
 * -----------------------------------------------------------------------------*/

// Initialize the menu items
void init_menu(void) {
    // Count how many menu items are enabled
    menu_item_count = 0;
    for (int i = 0; menu_item_configs[i].label != NULL; i++) {
        if (menu_item_configs[i].enabled) {
            menu_item_count++;
        }
    }

    // Allocate memory for the enabled menu items
    menu_items = (MenuItem *)malloc(menu_item_count * sizeof(MenuItem));

    if (!menu_items) {
        fprintf(stderr, "Memory allocation failed in init_menu()\n");
        exit(EXIT_FAILURE);
    }

    // Copy only the enabled menu items
    int menu_index = 0;
    for (int i = 0; menu_item_configs[i].label != NULL; i++) {
        if (menu_item_configs[i].enabled) {
            menu_items[menu_index].label = menu_item_configs[i].label;
            menu_items[menu_index].action = menu_item_configs[i].action;
            menu_index++;
        }
    }
    DEBUG_PRINT("Initialized menu with %d items\n", menu_item_count);
}

// Clean up menu memory
void cleanup_menu(void) {
    free(menu_items);
    menu_items = NULL;
    menu_item_count = 0;
}

//function to enable/disable menu items at runtime
void set_menu_item_enabled(int index, bool enabled) {
    // Make sure the index is valid
    int i;
    for (i = 0; menu_item_configs[i].label != NULL; i++) {
        if (i == index) {
            menu_item_configs[i].enabled = enabled;
            DEBUG_PRINT("Menu item %d (%s) %s\n",
                      i, menu_item_configs[i].label,
                      enabled ? "enabled" : "disabled");
            break;
        }
    }

    // If we didn't find the item, print a warning
    if (menu_item_configs[i].label == NULL) {
        DEBUG_PRINT("Warning: Tried to set state of invalid menu item %d\n", index);
    }
}

//function to reload the menu after changing configuration
void reload_menu(void) {
    // Free the old menu items
    cleanup_menu();

    // Initialize the menu again with the current configuration
    init_menu();

    // Reset the menu cursor position
    display_state.current_menu_item = 0;
    display_state.menu_scroll_offset = 0;

    // Redraw the menu
    update_menu_display();
}

// Update just the lines that need to change when selection changes
void update_selection(int old_selection, int new_selection) {
    // Check if either the old or new selection is currently visible
    bool old_visible = (old_selection >= display_state.menu_scroll_offset &&
                        old_selection < display_state.menu_scroll_offset + MENU_VISIBLE_ITEMS);
    bool new_visible = (new_selection >= display_state.menu_scroll_offset &&
                       new_selection < display_state.menu_scroll_offset + MENU_VISIBLE_ITEMS);

    // If the new selection isn't visible, we need to scroll and redraw the whole menu
    if (!new_visible) {
        // Full redraw will handle the scrolling
        update_menu_display();
        return;
    }
    // If the old selection isn't visible, just update the new selection
    if (!old_visible) {
        char buffer[32];
        int menu_pos = new_selection - display_state.menu_scroll_offset;
        // Format with the arrow indicator
        int ret = snprintf(buffer, sizeof(buffer), "> %s", menu_items[new_selection].label);
        if (ret < 0 || ret >= (int)sizeof(buffer)) {
            DEBUG_PRINT("Warning: Buffer truncation in update_selection()\n");
        }
        // Calculate position and draw
        int y_pos = MENU_START_Y + (menu_pos * MENU_ITEM_SPACING);
        send_draw_text(0, y_pos, buffer);
        DEBUG_PRINT("Updating selection: '%s' at y=%d\n", buffer, y_pos);
        return;
    }
    // Both old and new selection are visible, so just update those two lines
    // Update the old selection (remove the arrow)
    if (old_selection >= 0 && old_selection < menu_item_count) {
        char buffer[32];
        int menu_pos = old_selection - display_state.menu_scroll_offset;
        // Format without the arrow indicator
        int ret = snprintf(buffer, sizeof(buffer), "  %s", menu_items[old_selection].label);
        if (ret < 0 || ret >= (int)sizeof(buffer)) {
            DEBUG_PRINT("Warning: Buffer truncation in update_selection()\n");
        }
        // Calculate y position and draw
        int y_pos = MENU_START_Y + (menu_pos * MENU_ITEM_SPACING);
        send_draw_text(0, y_pos, buffer);
        usleep(DISPLAY_CMD_DELAY);
        DEBUG_PRINT("Updating line %d: '%s' at y=%d\n", old_selection + 1, buffer, y_pos);
    }

    // Update the new selection (add the arrow)
    if (new_selection >= 0 && new_selection < menu_item_count) {
        char buffer[32];
        int menu_pos = new_selection - display_state.menu_scroll_offset;

        // Format with the arrow indicator
        int ret = snprintf(buffer, sizeof(buffer), "> %s", menu_items[new_selection].label);
        if (ret < 0 || ret >= (int)sizeof(buffer)) {
            DEBUG_PRINT("Warning: Buffer truncation in update_selection()\n");
        }

        // Calculate y position and draw
        int y_pos = MENU_START_Y + (menu_pos * MENU_ITEM_SPACING);
        send_draw_text(0, y_pos, buffer);
        usleep(DISPLAY_CMD_DELAY);

        DEBUG_PRINT("Updating line %d: '%s' at y=%d\n", new_selection + 1, buffer, y_pos);
    }
}

// Update the menu display on the OLED with proper delays
void update_menu_display(void) {
    struct timeval now;
    gettimeofday(&now, NULL);

    // Calculate time diff in milliseconds since last update
    long time_diff_ms = get_time_diff_ms(&display_state.last_update_time, &now);

    // Debounce display updates (don't update more often than every 100ms)
    if (time_diff_ms < DISPLAY_UPDATE_DEBOUNCE) {
        display_state.needs_update = 1;
        return;
    }

    // If an update is already in progress, just mark that we need another one
    if (display_state.update_in_progress) {
        display_state.needs_update = 1;
        return;
    }

    display_state.update_in_progress = 1;
    display_state.needs_update = 0;

    // Calculate the scroll offset to ensure the selected item is visible
    if (display_state.current_menu_item < display_state.menu_scroll_offset) {
        // Selected item is above the visible area - scroll up
        display_state.menu_scroll_offset = display_state.current_menu_item;
    } else if (display_state.current_menu_item >= display_state.menu_scroll_offset + MENU_VISIBLE_ITEMS) {
        // Selected item is below the visible area - scroll down
        display_state.menu_scroll_offset = display_state.current_menu_item - MENU_VISIBLE_ITEMS + 1;
    }

    // Ensure scroll offset stays within valid range
    if (display_state.menu_scroll_offset < 0) {
        display_state.menu_scroll_offset = 0;
    }
    if (display_state.menu_scroll_offset > menu_item_count - MENU_VISIBLE_ITEMS) {
        display_state.menu_scroll_offset = menu_item_count - MENU_VISIBLE_ITEMS;
        if (display_state.menu_scroll_offset < 0) {
            display_state.menu_scroll_offset = 0;  // In case we have fewer items than visible slots
        }
    }

    DEBUG_PRINT("Updating display with menu item %d of %d (scroll offset: %d)\n",
                display_state.current_menu_item + 1, menu_item_count, display_state.menu_scroll_offset);

    // Clear the display first
    send_clear();
    usleep(DISPLAY_CLEAR_DELAY);  // 50ms delay after clear

    // Draw a title at the top
    send_draw_text(24, 0, MENU_TITLE);
    usleep(DISPLAY_CMD_DELAY);

    // Draw a separator line
    send_draw_text(0, 8, MENU_SEPARATOR);
    usleep(DISPLAY_CMD_DELAY);

    // Now draw visible menu items with proper spacing
    int displayed_items = 0;
    int max_items_to_display = menu_item_count < MENU_VISIBLE_ITEMS ?
                              menu_item_count : MENU_VISIBLE_ITEMS;

    for (int i = 0; i < max_items_to_display; i++) {
        int menu_index = i + display_state.menu_scroll_offset;

        // Skip if we've gone past the end of the menu
        if (menu_index >= menu_item_count) {
            break;
        }

        char buffer[32];
        // Format menu item text with selection indicator
        int ret;
        if (menu_index == display_state.current_menu_item) {
            ret = snprintf(buffer, sizeof(buffer), "> %s", menu_items[menu_index].label);
        } else {
            ret = snprintf(buffer, sizeof(buffer), "  %s", menu_items[menu_index].label);
        }

        if (ret < 0 || ret >= (int)sizeof(buffer)) {
            DEBUG_PRINT("Warning: Buffer truncation in update_menu_display()\n");
        }

        // Calculate y position based on menu start position and spacing
        int y_pos = MENU_START_Y + (i * MENU_ITEM_SPACING);
        send_draw_text(0, y_pos, buffer);
        displayed_items++;

        // Print debug info
        DEBUG_PRINT("Menu item %d: '%s' at y=%d\n", menu_index + 1, buffer, y_pos);

        // Add delay between drawing commands
        usleep(DISPLAY_CMD_DELAY);
    }

    // Draw scroll indicators if needed
    if (menu_item_count > MENU_VISIBLE_ITEMS) {
        // Draw up arrow if there are items above
        if (display_state.menu_scroll_offset > 0) {
            send_draw_text(DISPLAY_WIDTH - MENU_SCROLL_INDICATOR_WIDTH, MENU_START_Y, "^");
        }

        // Draw down arrow if there are items below
        if (display_state.menu_scroll_offset + MENU_VISIBLE_ITEMS < menu_item_count) {
            int y_pos = MENU_START_Y + ((MENU_VISIBLE_ITEMS - 1) * MENU_ITEM_SPACING);
            send_draw_text(DISPLAY_WIDTH - MENU_SCROLL_INDICATOR_WIDTH, y_pos, "v");
        }
    }

    // Make sure all commands are processed
    usleep(DISPLAY_CMD_DELAY * 2);

    // Update the timestamp
    gettimeofday(&display_state.last_update_time, NULL);

    display_state.update_in_progress = 0;

    // If another update was requested during this one, do it now
    if (display_state.needs_update) {
        usleep(DISPLAY_CMD_DELAY * 2);
        display_state.needs_update = 0;
        update_menu_display();
    }
}

/* -----------------------------------------------------------------------------
 * Input handling functions
 * -----------------------------------------------------------------------------*/

// Handle key press events with acceleration
void handle_key_left(void) {
    update_activity_timestamp();
    struct timeval now;
    gettimeofday(&now, NULL);

    // Calculate time diff in milliseconds
    long time_diff_ms = get_time_diff_ms(&input_state.last_key_time, &now);

    // Check if this is a rapid succession of the same key
    if (input_state.last_direction == 1 && time_diff_ms < KEY_ACCELERATION_THRESHOLD) {
        input_state.consecutive_same_key++;
    } else {
        input_state.consecutive_same_key = 0;
    }

    // Apply acceleration (move more items for fast rotation)
    int steps = 1;
    if (input_state.consecutive_same_key > 5) {
        steps = 3; // Move 3 items at a time when rotating quickly
    } else if (input_state.consecutive_same_key > 2) {
        steps = 2; // Move 2 items at a time
    }

    // Remember old selection
    int old_selection = display_state.current_menu_item;

    // Move the selection up
    for (int i = 0; i < steps; i++) {
        if (display_state.current_menu_item > 0) {
            display_state.current_menu_item--;
        }
    }

    // If the selection actually changed, update the display
    if (old_selection != display_state.current_menu_item) {
        update_selection(old_selection, display_state.current_menu_item);
    }

    input_state.last_direction = 1;
    input_state.last_key_time = now;
    DEBUG_PRINT("LEFT key pressed (steps: %d)\n", steps);
}

void handle_key_right(void) {
    update_activity_timestamp();
    struct timeval now;
    gettimeofday(&now, NULL);

    // Calculate time diff in milliseconds
    long time_diff_ms = get_time_diff_ms(&input_state.last_key_time, &now);

    // Check if this is a rapid succession of the same key
    if (input_state.last_direction == 2 && time_diff_ms < KEY_ACCELERATION_THRESHOLD) {
        input_state.consecutive_same_key++;
    } else {
        input_state.consecutive_same_key = 0;
    }

    // Apply acceleration (move more items for fast rotation)
    int steps = 1;
    if (input_state.consecutive_same_key > 5) {
        steps = 3; // Move 3 items at a time when rotating quickly
    } else if (input_state.consecutive_same_key > 2) {
        steps = 2; // Move 2 items at a time
    }

    // Remember old selection
    int old_selection = display_state.current_menu_item;

    // Move the selection down
    for (int i = 0; i < steps; i++) {
        if (display_state.current_menu_item < menu_item_count - 1) {
            display_state.current_menu_item++;
        }
    }

    // If the selection actually changed, update the display
    if (old_selection != display_state.current_menu_item) {
        update_selection(old_selection, display_state.current_menu_item);
    }

    input_state.last_direction = 2;
    input_state.last_key_time = now;
    DEBUG_PRINT("RIGHT key pressed (steps: %d)\n", steps);
}

/*void handle_key_enter(void) {
    update_activity_timestamp();
    DEBUG_PRINT("ENTER key pressed\n");
    if (display_state.current_menu_item >= 0 && display_state.current_menu_item < menu_item_count) {
        if (menu_items[display_state.current_menu_item].action) {
            menu_items[display_state.current_menu_item].action();
        }
    }
}*/
void handle_key_enter(void) {
    update_activity_timestamp();
    DEBUG_PRINT("ENTER key pressed\n");

    if (display_state.current_menu_item >= 0 && display_state.current_menu_item < menu_item_count) {
        if (menu_items[display_state.current_menu_item].action) {
            // Set the current submenu function
            current_submenu_function = menu_items[display_state.current_menu_item].action;
            exit_current_submenu = false;

            // Execute the function
            current_submenu_function();

            // Reset when done
            current_submenu_function = NULL;
        }
    }
}
// Handle all input events
void handle_input_events(void) {
    //handle power save timeout
    update_activity_timestamp();
    struct input_event ev;
    int event_count = 0;
    int btn_press = 0;
    bool pending_movement = false;
    struct timeval now;

    // Read and coalesce events
    while (read(input_fd, &ev, sizeof(ev)) > 0 && event_count < MAX_EVENTS_PER_ITERATION) {
        // Handle SYN_REPORT events (type 0)
        if (ev.type == 0) {
            // Skip SYN_REPORT events
            continue;
        }

        // Process based on event type
        if (ev.type == EV_REL && ev.code == REL_X) {
            // Get current time
            gettimeofday(&now, NULL);

            // Calculate time diff in milliseconds since last event
            long time_diff_ms = 0;
            if (input_state.last_event_time.tv_sec != 0) {
                time_diff_ms = get_time_diff_ms(&input_state.last_event_time, &now);
            }

            // Update the last event time
            input_state.last_event_time = now;

            // Reset paired count if this is a new movement after a long gap
            if (time_diff_ms > 100) {
                input_state.paired_event_count = 0;
                input_state.total_rel_x = 0;
            }

            // Always accumulate the value
            input_state.total_rel_x += ev.value;
            input_state.paired_event_count++;
            pending_movement = true;

            DEBUG_PRINT("REL_X event: value=%d, total=%d, paired=%d, diff=%ldms\n",
                   ev.value, input_state.total_rel_x, input_state.paired_event_count, time_diff_ms);

            event_count++;
        }
        else if (ev.type == EV_KEY && ev.code == BTN_LEFT && ev.value == 1) {
            // Mouse left button press (value 1 = pressed)
            btn_press = 1;
            event_count++;
        }

        // Avoid processing too many events at once
        if (event_count >= MAX_EVENTS_PER_ITERATION) {
            break;
        }
    }

    // Process button press if detected
    if (btn_press) {
        handle_key_enter();
    }

    // Now we handle the movement only if:
    // 1. We've received 2 or more events (paired_event_count >= 2), which means we've seen both events from one rotation
    // 2. OR if it's been more than 30ms since the last event, which means we might not get a paired event
    gettimeofday(&now, NULL);
    long time_since_last_ms = get_time_diff_ms(&input_state.last_event_time, &now);

    if (pending_movement && (input_state.paired_event_count >= 2 || time_since_last_ms > EVENT_PROCESS_THRESHOLD)) {
        // Only process if we have a non-zero total
        if (input_state.total_rel_x != 0) {
            // Process movement based on total relative movement
            if (input_state.total_rel_x < 0) {
                handle_key_left();
            } else if (input_state.total_rel_x > 0) {
                handle_key_right();
            }
        }

        // Reset tracking variables
        input_state.paired_event_count = 0;
        input_state.total_rel_x = 0;
    }

    // If we processed the maximum events, flush any remaining
    if (event_count >= MAX_EVENTS_PER_ITERATION) {
        // Drain input buffer
        while (read(input_fd, &ev, sizeof(ev)) > 0) {
            // Just drain, don't process
        }
    }
}

/* -----------------------------------------------------------------------------
 * Brightness control functions
 * -----------------------------------------------------------------------------*/

// Function to update only the changing parts of the brightness display
void update_brightness_value(int brightness) {
    char text[16];
    int percentage = (brightness * 100) / 255;

    // Update just the percentage text
    int ret = snprintf(text, sizeof(text), "%d%%", percentage);
    if (ret < 0 || ret >= (int)sizeof(text)) {
        DEBUG_PRINT("Warning: Buffer truncation in update_brightness_value()\n");
    }

    // Clear the previous text area first (draw empty spaces)
    send_draw_text(50, 20, "    "); // Clear previous value
    usleep(DISPLAY_CMD_DELAY);

    // Draw new value
    send_draw_text(50, 20, text);
    usleep(DISPLAY_CMD_DELAY);

    // Update the progress bar
    send_progress_bar(10, 30, 108, 15, percentage);

    // Set the actual display brightness
    send_brightness(brightness);

    DEBUG_PRINT("Brightness set to %d (%d%%)\n", brightness, percentage);
}

// Initial full screen setup for brightness control
void setup_brightness_screen(void) {
    // Clear display
    send_clear();
    usleep(DISPLAY_CMD_DELAY * 3);

    // Draw static elements
    send_draw_text(25, 5, "Brightness");
    usleep(DISPLAY_CMD_DELAY);
}

void brightness_control_loop(void) {
    int running_brightness_menu = 1;
    int pending_update = 1;
    struct timeval now;
    bool pending_movement = false;

    // Reset control variables
    input_state.consecutive_same_key = 0;
    input_state.last_direction = 0;

    DEBUG_PRINT("Entering brightness control mode\n");

    // Setup the initial screen once
    setup_brightness_screen();

    // Update the initial brightness values
    update_brightness_value(display_state.brightness);

    while (running_brightness_menu && running) {
        // Check for power save mode activation
        if (power_save_enabled) {
            check_power_save_timeout();
            if (!display_powered_on || power_save_activated) {
                DEBUG_PRINT("Power save detected- exiting\n");
                running_brightness_menu = 0;
                break;
            }
        }
    if (device_disconnected) {
            DEBUG_PRINT("Device disconnected during WiFi menu\n");
            running_brightness_menu = 0;
            break;
        }
        // Prepare select
        fd_set readfds;
        struct timeval tv;
        FD_ZERO(&readfds);
        FD_SET(input_fd, &readfds);

        // Set a short timeout
        tv.tv_sec = 0;
        tv.tv_usec = INPUT_SELECT_TIMEOUT; // 20ms timeout

        // Wait for events with timeout
        int ret = select(input_fd + 1, &readfds, NULL, NULL, &tv);

        if (ret > 0 && FD_ISSET(input_fd, &readfds)) {
            struct input_event ev;
            int event_count = 0;
            int btn_press = 0;
            pending_movement = false;

            // Read and coalesce events
            while (read(input_fd, &ev, sizeof(ev)) > 0 && event_count < MAX_EVENTS_PER_ITERATION) {
                // Handle SYN_REPORT events (type 0)
                if (ev.type == 0) {
                    continue;
                }

                // Process based on event type
                if (ev.type == EV_REL && ev.code == REL_X) {
                    // Get current time
                    gettimeofday(&now, NULL);

                    // Calculate time diff in milliseconds since last event
                    long time_diff_ms = 0;
                    if (input_state.last_event_time.tv_sec != 0) {
                        time_diff_ms = get_time_diff_ms(&input_state.last_event_time, &now);
                    }

                    // Update the last event time
                    input_state.last_event_time = now;

                    // Reset paired count if this is a new movement after a long gap
                    if (time_diff_ms > 100) {
                        input_state.paired_event_count = 0;
                        input_state.total_rel_x = 0;
                    }

                    // Always accumulate the value
                    input_state.total_rel_x += ev.value;
                    input_state.paired_event_count++;
                    pending_movement = true;

                    DEBUG_PRINT("REL_X event: value=%d, total=%d, paired=%d, diff=%ldms\n",
                           ev.value, input_state.total_rel_x, input_state.paired_event_count, time_diff_ms);

                    event_count++;
                }
                else if (ev.type == EV_KEY && ev.code == BTN_LEFT && ev.value == 1) {
                    // Mouse left button press (value 1 = pressed)
                    btn_press = 1;
                    event_count++;
                }

                // Avoid processing too many events at once
                if (event_count >= MAX_EVENTS_PER_ITERATION) {
                    break;
                }
            }

            // Process button press if detected
            if (btn_press) {
                update_activity_timestamp();
                DEBUG_PRINT("Button pressed, exiting brightness control\n");
                running_brightness_menu = 0;
            }

            // Now handle the movement for brightness control
            gettimeofday(&now, NULL);
            long time_since_last_ms = get_time_diff_ms(&input_state.last_event_time, &now);

            if (pending_movement && (input_state.paired_event_count >= 2 || time_since_last_ms > EVENT_PROCESS_THRESHOLD)) {
        update_activity_timestamp();
                // Only process if we have a non-zero total
                if (input_state.total_rel_x != 0) {
                    // Left/Right changes brightness
                    if (input_state.total_rel_x < 0) {
                        display_state.brightness -= 10;
                        if (display_state.brightness < 0) display_state.brightness = 0;
                        pending_update = 1;
                    } else if (input_state.total_rel_x > 0) {
                        display_state.brightness += 10;
                        if (display_state.brightness > 255) display_state.brightness = 255;
                        pending_update = 1;
                    }
                }

                // Reset tracking variables
                input_state.paired_event_count = 0;
                input_state.total_rel_x = 0;
                pending_movement = false;
            }

            // If we processed the maximum events, flush any remaining
            if (event_count >= MAX_EVENTS_PER_ITERATION) {
                // Drain input buffer
                while (read(input_fd, &ev, sizeof(ev)) > 0) {
                    // Just drain, don't process
                }
            }
        }

        // Update just the brightness value if needed (without clearing the screen)
        if (pending_update) {
            update_brightness_value(display_state.brightness);
            pending_update = 0;
        }

        // Check if command buffer needs flushing
        gettimeofday(&now, NULL);
        long time_since_flush = get_time_diff_ms(&cmd_buffer.last_flush, &now);
        if (time_since_flush > CMD_BUFFER_FLUSH_INTERVAL && cmd_buffer.used > 0) {
            flush_cmd_buffer();
        }

        // Small delay to reduce CPU usage
        usleep(MAIN_LOOP_DELAY); // 5ms
    }

    // When exiting the brightness menu, update the main menu
    update_menu_display();
}

// Brightness control action function
void action_brightness(void) {
    // Initialize and enter the brightness control loop
    brightness_control_loop();
}

/* -----------------------------------------------------------------------------
 * Network and System functions
 * -----------------------------------------------------------------------------*/
void action_network_settings(void) {
    char ip_str[64] = "Unknown";
    char mac_str[64] = "Unknown";
    char ifname_str[64] = "Unknown";
    char temp_str[128] = "Unknown";

    // Get network information
    get_network_info(ip_str, sizeof(ip_str), mac_str, sizeof(mac_str),ifname_str,sizeof(ifname_str));

    // Clear display and show network info
    send_clear();
    usleep(DISPLAY_CMD_DELAY * 3);

    // Draw the title
    send_draw_text(0, 0, "Network Setting");
    usleep(DISPLAY_CMD_DELAY);

    // Draw a separator
    send_draw_text(0, 8, "----------------");
    usleep(DISPLAY_CMD_DELAY);

    // Format IP with line breaks - first line shows label
    snprintf(temp_str,sizeof(temp_str),"IP:%s",ifname_str);
    send_draw_text(0, 16, temp_str);
    //send_draw_text(0, 16, "IP:");
    usleep(DISPLAY_CMD_DELAY);

    // Split IP into 15-char chunks and display on separate lines
    char *ip_ptr = ip_str;
    int y_pos = 24;
    int remaining = strlen(ip_str);

    while (remaining > 0) {
        char temp[16] = {0}; // 15 chars + null terminator
        int chunk = remaining > 15 ? 15 : remaining;

        strncpy(temp, ip_ptr, chunk);
        temp[chunk] = '\0';

        send_draw_text(0, y_pos, temp);
        usleep(DISPLAY_CMD_DELAY);

        ip_ptr += chunk;
        remaining -= chunk;
        y_pos += 10;
    }
    y_pos+=8;
    // Format MAC address - first line shows label
    send_draw_text(0, y_pos, "MAC:");
    usleep(DISPLAY_CMD_DELAY);
    y_pos += 8;

    // MAC address (should fit on one line, but just in case)
    send_draw_text(0, y_pos, mac_str);
    usleep(DISPLAY_CMD_DELAY);

    // Wait for user to press button to return to menu
    int running_network_menu = 1;

    DEBUG_PRINT("Displaying network settings...\n");

    while (running_network_menu && running) {
        // Check for power save mode activation
        if (power_save_enabled) {
            check_power_save_timeout();
            if (!display_powered_on || power_save_activated) {
                DEBUG_PRINT("Power save detected - exiting\n");
                running_network_menu = 0;
                break;
            }
        }
    if (device_disconnected) {
            DEBUG_PRINT("Device disconnected during WiFi menu\n");
            running_network_menu = 0;
            break;
        }
        // Prepare select
        fd_set readfds;
        struct timeval tv;
        FD_ZERO(&readfds);
        FD_SET(input_fd, &readfds);

        // Set a timeout
        tv.tv_sec = 0;
        tv.tv_usec = INPUT_SELECT_TIMEOUT;

        // Wait for events with timeout
        int ret = select(input_fd + 1, &readfds, NULL, NULL, &tv);

        if (ret > 0 && FD_ISSET(input_fd, &readfds)) {
            struct input_event ev;

            // Read events
            while (read(input_fd, &ev, sizeof(ev)) > 0) {
            update_activity_timestamp();
            // Check for button press
                if (ev.type == EV_KEY && ev.code == BTN_LEFT && ev.value == 1) {
                    running_network_menu = 0;
                    break;
                }
            }
        }

        // Small delay to reduce CPU usage
        usleep(MAIN_LOOP_DELAY);
    }

    // Return to main menu
    update_menu_display();
}

void get_network_info(char *ip_str, size_t ip_len, char *mac_str, size_t mac_len,char *if_str,size_t if_len) {
    struct ifaddrs *ifaddr, *ifa;
    int family, s;
    char host[NI_MAXHOST];
    bool found_ip = false;

    // Initialize with defaults
    snprintf(ip_str, ip_len, "Not connected");
    snprintf(mac_str, mac_len, "Not available");
    snprintf(if_str, if_len, "Not available");

    // Get IP address
    if (getifaddrs(&ifaddr) == -1) {
        DEBUG_PRINT("getifaddrs failed: %s\n", strerror(errno));
        return;
    }

    // First pass: Look for non-loopback interfaces
    for (ifa = ifaddr; ifa != NULL && !found_ip; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == NULL)
            continue;

        family = ifa->ifa_addr->sa_family;

        // Skip if not IPv4
        if (family != AF_INET)
            continue;

        // Skip loopback interfaces (lo)
        if (strcmp(ifa->ifa_name, "lo") == 0)
            continue;

        // Check if interface is up and running
        if (!(ifa->ifa_flags & IFF_UP) || !(ifa->ifa_flags & IFF_RUNNING))
            continue;

        s = getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in),
                       host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);

        if (s != 0) {
            DEBUG_PRINT("getnameinfo failed: %s\n", gai_strerror(s));
            continue;
        }

        //snprintf(ip_str, ip_len, "%s (%s)", host, ifa->ifa_name);
        snprintf(ip_str, ip_len, "%s", host);
        snprintf(if_str, if_len, "%s", ifa->ifa_name);
        found_ip = true;

        // Try to get MAC address for the same interface
        struct ifreq ifr;
        int fd = socket(AF_INET, SOCK_DGRAM, 0);

        if (fd >= 0) {
            strcpy(ifr.ifr_name, ifa->ifa_name);
            if (ioctl(fd, SIOCGIFHWADDR, &ifr) == 0) {
                unsigned char *mac = (unsigned char *)ifr.ifr_hwaddr.sa_data;
                snprintf(mac_str, mac_len, "%02X%02X%02X%02X%02X%02X",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        }
            close(fd);
        }
    }

    // If we didn't find any suitable interface, use loopback as fallback
    if (!found_ip) {
        for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
            if (ifa->ifa_addr == NULL)
                continue;

            family = ifa->ifa_addr->sa_family;

            if (family != AF_INET)
                continue;

            if (strcmp(ifa->ifa_name, "lo") == 0) {
                s = getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in),
                               host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);

                if (s != 0) {
                    DEBUG_PRINT("getnameinfo failed: %s\n", gai_strerror(s));
                    continue;
                }

                //snprintf(ip_str, ip_len, "%s (lo)", host);
                snprintf(ip_str, ip_len, "%s", host);
                snprintf(if_str, if_len, "lo");

                // Try to get MAC for loopback too, though it likely won't have one
                struct ifreq ifr;
                int fd = socket(AF_INET, SOCK_DGRAM, 0);

                if (fd >= 0) {
                    strcpy(ifr.ifr_name, ifa->ifa_name);
                    if (ioctl(fd, SIOCGIFHWADDR, &ifr) == 0) {
                        unsigned char *mac = (unsigned char *)ifr.ifr_hwaddr.sa_data;
                        snprintf(mac_str, mac_len, "%02X:%02X:%02X:%02X:%02X:%02X",
                                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
                    }
                    close(fd);
                }

                break;
            }
        }
    }

    freeifaddrs(ifaddr);
}

void action_system_stats(void) {
    char cpu_str[16] = "Unknown";
    char mem_total_str[32] = "Unknown";
    char mem_free_str[32] = "Unknown";
    int cpu_percentage = 0;
    int mem_percentage = 0;

    // Clear display and show initial system info
    send_clear();
    usleep(DISPLAY_CMD_DELAY * 3);
    send_draw_text(0, 0, "  System Stats");
    usleep(DISPLAY_CMD_DELAY);

    // Draw a separator
    send_draw_text(0, 8, "----------------");
    usleep(DISPLAY_CMD_DELAY);

    int running_stats_menu = 1;
    struct timeval last_update = {0, 0};

    DEBUG_PRINT("Displaying system stats...\n");

    // Configure Y positions for the display elements
    const int cpu_label_y = 16;
    const int cpu_bar_y = 25;
    const int mem_label_y = 42;//38;
    const int mem_bar_y = 51;//47;

    while (running_stats_menu && running) {
        // Check for power save mode activation
        if (power_save_enabled) {
            check_power_save_timeout();
            if (!display_powered_on || power_save_activated) {
                DEBUG_PRINT("Power save detected - exiting\n");
                running_stats_menu = 0;
                break;
            }
        }
        // Check if device was disconnected
        if (device_disconnected) {
            DEBUG_PRINT("Device disconnected during system stats display\n");
            running_stats_menu = 0;
            break;
        }
        // Update stats every STAT_UPDATE_SEC seconds
        struct timeval now;
        gettimeofday(&now, NULL);
        long time_diff_sec = (now.tv_sec - last_update.tv_sec);

        if (time_diff_sec >= STAT_UPDATE_SEC || last_update.tv_sec == 0) {
            // Get updated system information with percentages
            get_system_info_with_percent(cpu_str, sizeof(cpu_str), mem_total_str, sizeof(mem_total_str),
                                     mem_free_str, sizeof(mem_free_str), &cpu_percentage, &mem_percentage);

            // Update CPU label and value on the same line
            send_draw_text(0, cpu_label_y, "CPU:");
            send_draw_text(40, cpu_label_y, "    ");  // Clear old value
            send_draw_text(40, cpu_label_y, cpu_str);

            // CPU progress bar on the next line (full width)
            send_progress_bar(0, cpu_bar_y, 128, 10, cpu_percentage);

            // Update Memory label and value on the same line
            send_draw_text(0, mem_label_y, "Memory:");
            send_draw_text(55, mem_label_y, "    ");  // Clear old value
            send_draw_text(55, mem_label_y, mem_total_str);

            // Memory progress bar on the next line (full width)
            send_progress_bar(0, mem_bar_y, 128, 10, mem_percentage);

            // Store current time as last update time
            last_update = now;
        }

        // Prepare select
        fd_set readfds;
        struct timeval tv;
        FD_ZERO(&readfds);
        FD_SET(input_fd, &readfds);

        // Set a short timeout to allow regular updates
        tv.tv_sec = 0;
        tv.tv_usec = 100000; // 100ms

        // Wait for events with timeout
        int ret = select(input_fd + 1, &readfds, NULL, NULL, &tv);

        if (ret > 0 && FD_ISSET(input_fd, &readfds)) {
            struct input_event ev;

            // Read events
            while (read(input_fd, &ev, sizeof(ev)) > 0) {
                update_activity_timestamp();
                // Check for button press
                if (ev.type == EV_KEY && ev.code == BTN_LEFT && ev.value == 1) {
                    running_stats_menu = 0;
                    break;
                }
            }
        }

        // Flush any pending commands
        if (cmd_buffer.used > 0) {
            flush_cmd_buffer();
        }

        // Small delay to reduce CPU usage
        usleep(MAIN_LOOP_DELAY);
    }

    // Return to main menu
    update_menu_display();
}

void get_system_info(char *cpu_str, size_t cpu_len, char *mem_total_str, size_t mem_total_len,
                    char *mem_free_str, size_t mem_free_len) {
    // Get CPU usage - this is a simple approximation
    static long prev_idle = 0;
    static long prev_total = 0;

    FILE *fp = fopen("/proc/stat", "r");
    if (fp) {
        long user, nice, system, idle, iowait, irq, softirq;

        // Read CPU statistics
        if (fscanf(fp, "cpu %ld %ld %ld %ld %ld %ld %ld",
                  &user, &nice, &system, &idle, &iowait, &irq, &softirq) == 7) {

            long total = user + nice + system + idle + iowait + irq + softirq;
            long total_diff = total - prev_total;
            long idle_diff = idle - prev_idle;

            // Calculate CPU usage percentage
            int cpu_usage = 0;
            if (total_diff > 0) {
                cpu_usage = (int)(100 * (total_diff - idle_diff) / total_diff);
            }

            // Update previous values for next calculation
            prev_idle = idle;
            prev_total = total;

            snprintf(cpu_str, cpu_len, "%d%%", cpu_usage);
        } else {
            snprintf(cpu_str, cpu_len, "Error");
        }

        fclose(fp);
    } else {
        snprintf(cpu_str, cpu_len, "Error");
    }

    // Get memory usage
    struct sysinfo info;
    if (sysinfo(&info) == 0) {
        // Calculate memory usage
        long total_ram = info.totalram * info.mem_unit;
        long free_ram = info.freeram * info.mem_unit;
        long used_ram = total_ram - free_ram;
        int mem_usage = (int)(100 * used_ram / total_ram);

        // Convert to human-readable format (GB)
        float total_gb = (float)total_ram / (1024 * 1024 * 1024);
        float free_gb = (float)free_ram / (1024 * 1024 * 1024);

        // Format strings
        snprintf(mem_total_str, mem_total_len, "%.1f GB (%d%%)", total_gb, mem_usage);
        snprintf(mem_free_str, mem_free_len, "%.1f GB (%d%%)", free_gb, 100 - mem_usage);
    } else {
        snprintf(mem_total_str, mem_total_len, "Error");
        snprintf(mem_free_str, mem_free_len, "Error");
    }
}

/* get system info with percentage values */
void get_system_info_with_percent(char *cpu_str, size_t cpu_len, char *mem_total_str, size_t mem_total_len,
                               char *mem_free_str, size_t mem_free_len, int *cpu_percentage, int *mem_percentage) {
    // Get CPU usage - this is a simple approximation
    static long prev_idle = 0;
    static long prev_total = 0;

    FILE *fp = fopen("/proc/stat", "r");
    if (fp) {
        long user, nice, system, idle, iowait, irq, softirq;

        // Read CPU statistics
        if (fscanf(fp, "cpu %ld %ld %ld %ld %ld %ld %ld",
                  &user, &nice, &system, &idle, &iowait, &irq, &softirq) == 7) {

            long total = user + nice + system + idle + iowait + irq + softirq;
            long total_diff = total - prev_total;
            long idle_diff = idle - prev_idle;

            // Calculate CPU usage percentage
            *cpu_percentage = 0;
            if (total_diff > 0) {
                *cpu_percentage = (int)(100 * (total_diff - idle_diff) / total_diff);
            }

            // Update previous values for next calculation
            prev_idle = idle;
            prev_total = total;

            snprintf(cpu_str, cpu_len, "%d%%", *cpu_percentage);
        } else {
            snprintf(cpu_str, cpu_len, "Err");
            *cpu_percentage = 0;
        }

        fclose(fp);
    } else {
        snprintf(cpu_str, cpu_len, "Err");
        *cpu_percentage = 0;
    }

    // Get memory usage
    struct sysinfo info;
    if (sysinfo(&info) == 0) {
        // Calculate memory usage
        long total_ram = info.totalram * info.mem_unit;
        long free_ram = info.freeram * info.mem_unit;
        long used_ram = total_ram - free_ram;
        *mem_percentage = (int)(100 * used_ram / total_ram);

        // Format strings
        snprintf(mem_total_str, mem_total_len, "%d%%", *mem_percentage);
        snprintf(mem_free_str, mem_free_len, "%d%%", 100 - *mem_percentage);
    } else {
        snprintf(mem_total_str, mem_total_len, "Err");
        snprintf(mem_free_str, mem_free_len, "Err");
        *mem_percentage = 0;
    }
}

void action_test_internet(void) {
    const char *server = "8.8.8.8";  // Google's DNS server
    const int timeout_sec = 5;        // 5-second timeout
    int test_result = -1;
    int progress = 0;
    bool test_completed = false;

    // Clear display and show initial screen
    send_clear();
    usleep(DISPLAY_CMD_DELAY * 3);
    send_draw_text(0, 0, " Internet Test");
    usleep(DISPLAY_CMD_DELAY);

    // Draw a separator
    send_draw_text(0, 8, "----------------");
    usleep(DISPLAY_CMD_DELAY);

    // Draw the testing message
    send_draw_text(20, 20, "Testing");
    usleep(DISPLAY_CMD_DELAY);

    // Initial progress bar (0%)
    send_progress_bar(10, 35, 108, 15, 0);
    usleep(DISPLAY_CMD_DELAY);

    // Record start time
    time_t start_time = time(NULL);

    // Create a child process to perform the ping test to avoid blocking the UI
    pid_t child_pid = fork();

    if (child_pid == 0) {
        // This is the child process - do the ping test
        exit(ping_server(server, timeout_sec));
    } else if (child_pid < 0) {
        // Fork failed
        send_draw_text(25, 60, "Test Error");
        DEBUG_PRINT("Fork failed: %s\n", strerror(errno));
        sleep(2);
        update_menu_display();
        return;
    }

    // Parent process continues here to update UI
    int running_test_menu = 1;

    while (running_test_menu && running) {
        // Check for power save mode activation - ADD THIS BLOCK
        if (power_save_enabled) {
            check_power_save_timeout();
            if (!display_powered_on || power_save_activated) {
                DEBUG_PRINT("Power save detected - exiting\n");
                running_test_menu = 0;
                break;
            }
        }

        if (device_disconnected) {
            DEBUG_PRINT("Device disconnected during WiFi menu\n");
            running_test_menu = 0;
            break;
        }
        // Check if test has completed
        if (!test_completed) {
            // Check if the child process has finished
            int status;
            pid_t result = waitpid(child_pid, &status, WNOHANG);

            if (result == child_pid) {
                // Child process has finished
                test_completed = true;
                if (WIFEXITED(status)) {
                    test_result = WEXITSTATUS(status);
                } else {
                    test_result = -1;  // Abnormal termination
                }

                // Update progress bar to 100%
                send_progress_bar(10, 35, 108, 15, 100);

                // Show result message
                if (test_result == 0) {
                    send_draw_text(0, 60, "                ");  // Clear line
                    send_draw_text(30, 60, "Connected!");
                } else {
                    send_draw_text(0, 60, "                ");  // Clear line
                    send_draw_text(20, 60, "No Connection");
                }
            } else {
                // Test still running, update progress bar
                time_t current_time = time(NULL);
                time_t elapsed = current_time - start_time;

                // Calculate progress percentage (based on timeout)
                progress = (elapsed * 100) / timeout_sec;
                if (progress > 95) progress = 95;  // Cap at 95% until complete

                // Update the progress bar
                send_progress_bar(10, 35, 108, 15, progress);

                // Add animated dots to the "Testing Internet" message
                int dots = (elapsed % 4);
                char message[20];
                strcpy(message, "Testing");
                for (int i = 0; i < dots; i++) {
                    strcat(message, ".");
                }
                send_draw_text(0, 20, "                ");  // Clear line
                send_draw_text(20, 20, message);
            }
        }

        // Check for button press to exit
        fd_set readfds;
        struct timeval tv;
        FD_ZERO(&readfds);
        FD_SET(input_fd, &readfds);

        // Set a short timeout
        tv.tv_sec = 0;
        tv.tv_usec = 100000; // 100ms

        int ret = select(input_fd + 1, &readfds, NULL, NULL, &tv);

        if (ret > 0 && FD_ISSET(input_fd, &readfds)) {
            struct input_event ev;

            while (read(input_fd, &ev, sizeof(ev)) > 0) {
                update_activity_timestamp();
                if (ev.type == EV_KEY && ev.code == BTN_LEFT && ev.value == 1) {
                    running_test_menu = 0;
                    break;
                }
            }
        }

        // Flush any pending commands
        if (cmd_buffer.used > 0) {
            flush_cmd_buffer();
        }

        // Small delay
        usleep(MAIN_LOOP_DELAY);
    }

    // If we exit before test is complete, kill the child process
    if (!test_completed && child_pid > 0) {
        kill(child_pid, SIGTERM);
        waitpid(child_pid, NULL, 0);  // Clean up zombie process
    }

    // Return to main menu
    update_menu_display();
}

/* Function to ping a server and return success (0) or failure (1) */
int ping_server(const char *server, int timeout_sec) {
    char command[100];

    //following sleep is to test progressbar effect
    //usleep(1000000);
    //usleep(1000000);
    //usleep(1000000);
    //usleep(1000000);

    // Construct ping command with timeout and minimal output
    snprintf(command, sizeof(command), "ping -c 1 -W %d %s > /dev/null 2>&1",
             timeout_sec, server);

    // Execute the command
    int result = system(command);

    // Return 0 for success, 1 for failure
    return (result == 0) ? 0 : 1;
}

int get_wifi_status(void) {
    static int wifi_state = 0;  // Default to OFF
    return wifi_state;
}

// Dummy function to set WiFi status
void set_wifi_status(int state) {
    static int wifi_state = 0;  // Internal state

    if (state != wifi_state) {
        // Only change state if different
        wifi_state = state;
        // Log the state change (in real implementation, control WiFi here)
        DEBUG_PRINT("WiFi state changed to %s\n", state ? "ON" : "OFF");
        // In a real implementation, you would control the actual WiFi:
        // Example:
        // if (state) {
        //     system("systemctl start wpa_supplicant");
        // } else {
        //     system("systemctl stop wpa_supplicant");
        // }
    }
}

// WiFi submenu implementation
void action_wifi_menu(void) {
    // WiFi submenu options
    const char *options[] = {
        "Turn On",
        "Turn Off",
        "Exit"
    };
    const int num_options = 3;
    int selected_option = 0;
    int current_wifi_state = get_wifi_status();

    // Clear display and show submenu
    send_clear();
    usleep(DISPLAY_CMD_DELAY * 3);
    send_draw_text(0, 0, " WiFi Settings");
    usleep(DISPLAY_CMD_DELAY);

    // Draw a separator
    send_draw_text(0, 8, "----------------");
    usleep(DISPLAY_CMD_DELAY);

    // Draw initial menu options - starting right after the separator with no status line
    for (int i = 0; i < num_options; i++) {
        char buffer[32];
        int y_pos = 16 + (i * 10);  // Start at y=16 with 10px spacing between items

        // Determine if this option represents current state
        bool is_current_state = (i == 0 && current_wifi_state) ||
                               (i == 1 && !current_wifi_state);

        // Format with selection indicator and/or inverse highlight
        if (i == selected_option) {
            if (is_current_state) {
                // Draw as inverse (current state and selected)
                snprintf(buffer, sizeof(buffer), "> [%s]", options[i]);
            } else {
                // Selected but not current state
                snprintf(buffer, sizeof(buffer), "> %s", options[i]);
            }
        } else {
            if (is_current_state) {
                // Current state but not selected
                snprintf(buffer, sizeof(buffer), "  [%s]", options[i]);
            } else {
                // Neither selected nor current state
                snprintf(buffer, sizeof(buffer), "  %s", options[i]);
            }
        }

        send_draw_text(0, y_pos, buffer);
        usleep(DISPLAY_CMD_DELAY);
    }

    // Submenu interaction loop
    int running_wifi_menu = 1;
    //struct timeval last_key_time = {0, 0};
    int paired_event_count = 0;
    int total_rel_x = 0;
    bool pending_movement = false;
    struct timeval last_event_time = {0, 0};

    while (running_wifi_menu && running) {
    // Check if we should exit due to power save
    if (exit_current_submenu||power_save_activated) {
        DEBUG_PRINT("Power save detected - exiting\n");
            running_wifi_menu = 0;
            break;
        }
       // Check if device was disconnected
        if (device_disconnected) {
           DEBUG_PRINT("Device disconnected during WiFi menu\n");
           running_wifi_menu = 0;
           break;
        }
        // Prepare select
        fd_set readfds;
        struct timeval tv;
        FD_ZERO(&readfds);
        FD_SET(input_fd, &readfds);

        // Set a short timeout
        tv.tv_sec = 0;
        tv.tv_usec = INPUT_SELECT_TIMEOUT;

        // Wait for events with timeout
        int ret = select(input_fd + 1, &readfds, NULL, NULL, &tv);

        if (ret > 0 && FD_ISSET(input_fd, &readfds)) {
            struct input_event ev;
            int event_count = 0;
            int btn_press = 0;

            // Read and coalesce events
            while (read(input_fd, &ev, sizeof(ev)) > 0 && event_count < MAX_EVENTS_PER_ITERATION) {
                // Handle SYN_REPORT events (type 0)
                if (ev.type == 0) {
                    // Skip SYN_REPORT events
                    continue;
                }

                // Process based on event type
                if (ev.type == EV_REL && ev.code == REL_X) {
                    // Get current time
                    struct timeval now;
                    gettimeofday(&now, NULL);

                    // Calculate time diff in milliseconds since last event
                    long time_diff_ms = 0;
                    if (last_event_time.tv_sec != 0) {
                        time_diff_ms = get_time_diff_ms(&last_event_time, &now);
                    }

                    // Update the last event time
                    last_event_time = now;

                    // Reset paired count if this is a new movement after a long gap
                    if (time_diff_ms > 100) {
                        paired_event_count = 0;
                        total_rel_x = 0;
                    }

                    // Always accumulate the value
                    total_rel_x += ev.value;
                    paired_event_count++;
                    pending_movement = true;

                    DEBUG_PRINT("WiFi menu: REL_X event: value=%d, total=%d, paired=%d\n",
                           ev.value, total_rel_x, paired_event_count);

                    event_count++;
                }
                else if (ev.type == EV_KEY && ev.code == BTN_LEFT && ev.value == 1) {
                    // Mouse left button press (value 1 = pressed)
                    btn_press = 1;
                    event_count++;
                }

                // Avoid processing too many events at once
                if (event_count >= MAX_EVENTS_PER_ITERATION) {
                    break;
                }
            }

            // Now we handle the movement only if:
            // 1. We've received 2 or more events (paired_event_count >= 2), which means we've seen both events from one rotation
            // 2. OR if it's been more than 30ms since the last event, which means we might not get a paired event
            struct timeval now;
            gettimeofday(&now, NULL);
            long time_since_last_ms = get_time_diff_ms(&last_event_time, &now);

            if (pending_movement && (paired_event_count >= 2 || time_since_last_ms > EVENT_PROCESS_THRESHOLD)) {
                update_activity_timestamp();
                // Only process if we have a non-zero total
                if (total_rel_x != 0) {
                    // Remember old selection for redraw
                    int old_selected = selected_option;

                    // Update selection based on rotation
                    if (total_rel_x < 0) {
                        // Move up in the menu
                        selected_option = (selected_option > 0) ? selected_option - 1 : 0;
                        DEBUG_PRINT("WiFi menu: Moving UP to option %d\n", selected_option);
                    } else {
                        // Move down in the menu
                        selected_option = (selected_option < num_options - 1) ?
                                         selected_option + 1 : num_options - 1;
                        DEBUG_PRINT("WiFi menu: Moving DOWN to option %d\n", selected_option);
                    }

                    // Only redraw if selection changed
                    if (old_selected != selected_option) {
                        // Redraw the changed menu items
                        for (int i = 0; i < num_options; i++) {
                            if (i == old_selected || i == selected_option) {
                                char buffer[32] = {0};  // Initialize to zeros
                                int y_pos = 16 + (i * 10);

                                // Determine state highlighting
                                bool is_current_state = (i == 0 && current_wifi_state) ||
                                                     (i == 1 && !current_wifi_state);

                                // Clear the line first to avoid display artifacts
                                send_draw_text(0, y_pos, "                    ");
                                usleep(DISPLAY_CMD_DELAY);

                                // Format with selection indicator and/or inverse highlight
                                if (i == selected_option) {
                                    if (is_current_state) {
                                        snprintf(buffer, sizeof(buffer), "> [%s]", options[i]);
                                    } else {
                                        snprintf(buffer, sizeof(buffer), "> %s", options[i]);
                                    }
                                } else {
                                    if (is_current_state) {
                                        snprintf(buffer, sizeof(buffer), "  [%s]", options[i]);
                                    } else {
                                        snprintf(buffer, sizeof(buffer), "  %s", options[i]);
                                    }
                                }

                                send_draw_text(0, y_pos, buffer);
                                usleep(DISPLAY_CMD_DELAY);
                            }
                        }
                    }
                }

                // Reset tracking variables
                paired_event_count = 0;
                total_rel_x = 0;
                pending_movement = false;
            }

            // If we processed the maximum events, flush any remaining
            if (event_count >= MAX_EVENTS_PER_ITERATION) {
                // Drain input buffer
                while (read(input_fd, &ev, sizeof(ev)) > 0) {
                    // Just drain, don't process
                }
            }

            if (btn_press) {
                update_activity_timestamp();
                // Handle button press based on selected option
                if (selected_option == 2) {
                    // Exit option selected
                    running_wifi_menu = 0;  // Exit the submenu
                } else {
                    // Try to change WiFi state
                    int new_state = (selected_option == 0) ? 1 : 0;

                    // Only act if state would change
                    if (new_state != current_wifi_state) {
                        // Update the state
                        set_wifi_status(new_state);
                        current_wifi_state = new_state;

                        // Redraw all options to update state indicators
                        for (int i = 0; i < num_options; i++) {
                            char buffer[32] = {0};  // Initialize to zeros
                            int y_pos = 16 + (i * 10);

                            // Clear the line first
                            send_draw_text(0, y_pos, "                    ");
                            usleep(DISPLAY_CMD_DELAY);

                            // Determine state highlighting
                            bool is_current_state = (i == 0 && current_wifi_state) ||
                                                 (i == 1 && !current_wifi_state);

                            // Format with selection indicator and/or inverse highlight
                            if (i == selected_option) {
                                if (is_current_state) {
                                    snprintf(buffer, sizeof(buffer), "> [%s]", options[i]);
                                } else {
                                    snprintf(buffer, sizeof(buffer), "> %s", options[i]);
                                }
                            } else {
                                if (is_current_state) {
                                    snprintf(buffer, sizeof(buffer), "  [%s]", options[i]);
                                } else {
                                    snprintf(buffer, sizeof(buffer), "  %s", options[i]);
                                }
                            }

                            send_draw_text(0, y_pos, buffer);
                            usleep(DISPLAY_CMD_DELAY);
                        }
                    }
                }
            }
        }

        // Flush any pending commands
        if (cmd_buffer.used > 0) {
            flush_cmd_buffer();
        }

        // Small delay to reduce CPU usage
        usleep(MAIN_LOOP_DELAY);

    // Check power save timeout
    if (power_save_enabled) {
            check_power_save_timeout();
    }
    }

    // Return to main menu
    update_menu_display();
}

/* Update the detect_and_run function to include monitoring */
void detect_and_run(void) {
    while (running) {
        printf("Starting HMI device detection...\n");
        printf("Looking for: %s by %s (VID:PID %s:%s)\n",
               HMI_PRODUCT_NAME, HMI_MANUFACTURER, HMI_VENDOR_ID, HMI_PRODUCT_ID);

        // Reset disconnection flag
        device_disconnected = false;

        // First check if device is already connected
        if (!check_device_present()) {
            printf("HMI device not found. Monitoring for device connection...\n");
            monitor_device_until_connected();
        }

        // At this point, the device should be connected
        printf("HMI device detected!\n");

        // Find the correct device files
        char* input_device = find_hmi_input_device();
        char* serial_device = find_hmi_serial_device();

        if (!input_device || !serial_device) {
            fprintf(stderr, "Failed to find all required device files\n");
            free(input_device);
            free(serial_device);
            sleep(2);  // Wait a bit before trying again
            continue;
        }

        printf("Found input device: %s\n", input_device);
        printf("Found serial device: %s\n", serial_device);

        // Update config with found devices
        config.input_device = input_device;
        config.serial_device = serial_device;

        // Open devices
        input_fd = open_input_device(config.input_device);
        if (input_fd < 0) {
            free(input_device);
            free(serial_device);
            fprintf(stderr, "Failed to open input device\n");
            sleep(2);  // Wait a bit before trying again
            continue;
        }

        serial_fd = open_serial_device(config.serial_device);
        if (serial_fd < 0) {
            close(input_fd);
            free(input_device);
            free(serial_device);
            fprintf(stderr, "Failed to open serial device\n");
            sleep(2);  // Wait a bit before trying again
            continue;
        }

        // Initialize the menu
        init_menu();

        // Initial display with extra delay to make sure device is fully initialized
        usleep(STARTUP_DELAY); // 1s delay to give the device time to initialize
        printf("Initializing display...\n");

        // First clear the display
        send_clear();
        usleep(DISPLAY_CMD_DELAY * 15); // Very long delay after initial clear (150ms)

        // Draw a startup message (one line at a time with long delays)
        send_draw_text(0, 0, "Menu System");
        usleep(DISPLAY_CMD_DELAY * 10); // 100ms delay

        // Send one command at a time with long delays
        send_draw_text(0, 10, "Initializing...");
        usleep(DISPLAY_CMD_DELAY * 10); // 100ms delay

        // One more clear before showing the menu
        send_clear();
        usleep(DISPLAY_CMD_DELAY * 15); // 150ms delay

    // Initialize the activity timestamp to the current time
    gettimeofday(&last_activity_time, NULL);
    DEBUG_PRINT("Activity timestamp initialized\n");
    // If power save is enabled, ensure the display is powered on initially
    if (power_save_enabled) {
        display_powered_on = true;
            // Send explicit power-on command to ensure hardware state matches our variable
            unsigned char cmd[2];
            cmd[0] = CMD_POWER_MODE;
            cmd[1] = 0x01;  // Power ON
            send_command(cmd, 2);
        DEBUG_PRINT("Display power state initialized to ON\n");
    }

    // Now draw the menu
        update_menu_display();

        // Initialize timeval for key handling
        gettimeofday(&input_state.last_key_time, NULL);
        gettimeofday(&display_state.last_update_time, NULL);
        gettimeofday(&cmd_buffer.last_flush, NULL);

        // Start the disconnection monitor thread
        monitor_thread_running = true;
        pthread_create(&disconnect_monitor_tid, NULL, disconnection_monitor_thread, NULL);

        // Main event loop
        struct timeval now;
        bool local_running = true;

        while (local_running && running) {
            // Check if device was disconnected
            if (device_disconnected) {
                printf("Device disconnection detected! Returning to detection mode...\n");
                local_running = false;
                break;
            }

            // Prepare select
            fd_set readfds;
            struct timeval tv;
            FD_ZERO(&readfds);
            FD_SET(input_fd, &readfds);

            // Set a short timeout
            tv.tv_sec = 0;
            tv.tv_usec = INPUT_SELECT_TIMEOUT; // 20ms timeout

            // Wait for events with timeout
            int ret = select(input_fd + 1, &readfds, NULL, NULL, &tv);

            // Check for select errors which might indicate device disconnection
            if (ret < 0 && errno != EINTR) {
                printf("Select error: %s - device may be disconnected\n", strerror(errno));
                local_running = false;
                break;
            }

            if (ret > 0 && FD_ISSET(input_fd, &readfds)) {
                // Try to handle input - if device is gone, read will fail
                //struct input_event ev;
                //if (read(input_fd, &ev, sizeof(ev)) <= 0 && errno != EAGAIN) {
                //    printf("Input device read error: %s - device disconnected\n", strerror(errno));
                //    local_running = false;
                //    break;
                //}

                // If read succeeded, process the event
                handle_input_events();
            }

            // Process pending display updates if needed
            if (display_state.needs_update && !display_state.update_in_progress) {
                gettimeofday(&now, NULL);

                // Calculate time diff in milliseconds
                long time_diff_ms = get_time_diff_ms(&display_state.last_update_time, &now);

                // Only update if enough time has passed
                if (time_diff_ms > DISPLAY_UPDATE_DEBOUNCE) {
                    update_menu_display();
                }
            }

            // Check if command buffer needs flushing
            gettimeofday(&now, NULL);
            long time_since_flush = get_time_diff_ms(&cmd_buffer.last_flush, &now);
            if (time_since_flush > CMD_BUFFER_FLUSH_INTERVAL && cmd_buffer.used > 0) {
                flush_cmd_buffer();
            }

        //check_power_save_timeout();
        check_power_save_and_signal_exit();

            // Small delay to reduce CPU usage
            usleep(MAIN_LOOP_DELAY); // 5ms
        }

        // Stop the monitor thread
        monitor_thread_running = false;
        pthread_join(disconnect_monitor_tid, NULL);

        // Clean up before restarting
        send_clear();
        usleep(DISPLAY_CMD_DELAY);
        send_draw_text(0, 0, "Device disconnected");
        usleep(DISPLAY_CMD_DELAY * 10);

        cleanup_menu();

        if (input_fd >= 0) {
            close(input_fd);
            input_fd = -1;
        }

        if (serial_fd >= 0) {
            close(serial_fd);
            serial_fd = -1;
        }

        // Free allocated memory
        free(input_device);
        free(serial_device);

        // Wait a moment before restarting detection
        sleep(1);
    }

    // Final cleanup
    pthread_mutex_destroy(&serial_mutex);
}

/* Check if the HMI device is already connected */
bool check_device_present(void) {
    struct udev *udev;
    struct udev_enumerate *enumerate;
    struct udev_list_entry *devices, *dev_list_entry;
    bool found = false;

    // Create udev context
    udev = udev_new();
    if (!udev) {
        fprintf(stderr, "Failed to create udev context\n");
        return false;
    }

    // Create enumerate object
    enumerate = udev_enumerate_new(udev);
    udev_enumerate_add_match_subsystem(enumerate, "usb");
    udev_enumerate_scan_devices(enumerate);

    // Get the list of matching devices
    devices = udev_enumerate_get_list_entry(enumerate);

    // Iterate through devices
    udev_list_entry_foreach(dev_list_entry, devices) {
        const char *path = udev_list_entry_get_name(dev_list_entry);
        struct udev_device *dev = udev_device_new_from_syspath(udev, path);

        if (dev) {
            const char *vendor = udev_device_get_sysattr_value(dev, "idVendor");
            const char *product = udev_device_get_sysattr_value(dev, "idProduct");
            const char *manufacturer = udev_device_get_sysattr_value(dev, "manufacturer");
            const char *product_name = udev_device_get_sysattr_value(dev, "product");

            // Check if this is our HMI device
            if (vendor && product &&
                strcmp(vendor, HMI_VENDOR_ID) == 0 &&
                strcmp(product, HMI_PRODUCT_ID) == 0) {

                if (manufacturer && product_name &&
                    strstr(manufacturer, HMI_MANUFACTURER) != NULL &&
                    strstr(product_name, HMI_PRODUCT_NAME) != NULL) {

                    printf("Found device: %s %s (VID:PID %s:%s)\n",
                           manufacturer, product_name, vendor, product);
                    found = true;
                }
            }

            udev_device_unref(dev);

            if (found) break;
        }
    }

    // Clean up
    udev_enumerate_unref(enumerate);
    udev_unref(udev);

    return found;
}

/* Monitor for device connection */
void monitor_device_until_connected(void) {
    struct udev *udev;
    struct udev_monitor *mon;
    int fd;

    // Create udev context
    udev = udev_new();
    if (!udev) {
        fprintf(stderr, "Failed to create udev context\n");
        return;
    }

    // Set up monitoring
    mon = udev_monitor_new_from_netlink(udev, "udev");
    udev_monitor_filter_add_match_subsystem_devtype(mon, "usb", NULL);
    udev_monitor_enable_receiving(mon);
    fd = udev_monitor_get_fd(mon);

    // Set up polling
    struct pollfd fds[1];
    fds[0].fd = fd;
    fds[0].events = POLLIN;

    printf("Waiting for HMI device to be connected...\n");

    while (1) {
        // First check if device is already present
        if (check_device_present()) {
            printf("HMI device found!\n");
            break;
        }

        // Wait for device events
        int ret = poll(fds, 1, UDEV_POLL_TIMEOUT);

        if (ret > 0 && (fds[0].revents & POLLIN)) {
            // Get the device
            struct udev_device *dev = udev_monitor_receive_device(mon);
            if (dev) {
                const char *action = udev_device_get_action(dev);

                // Only care about add events
                if (action && strcmp(action, "add") == 0) {
                    // Check if this is potentially our device
                    struct udev_device *usb_dev = udev_device_get_parent_with_subsystem_devtype(
                        dev, "usb", "usb_device");

                    if (usb_dev) {
                        const char *vendor = udev_device_get_sysattr_value(usb_dev, "idVendor");
                        const char *product = udev_device_get_sysattr_value(usb_dev, "idProduct");

                        if (vendor && product &&
                            strcmp(vendor, HMI_VENDOR_ID) == 0 &&
                            strcmp(product, HMI_PRODUCT_ID) == 0) {

                            printf("USB device connected (VID:PID %s:%s)\n", vendor, product);

                            // Give some time for all device nodes to be created
                            sleep(2);

                            // Double check that all required interfaces are present
                            if (check_device_present()) {
                                printf("HMI device fully initialized!\n");
                                udev_device_unref(dev);
                                break;
                            }
                        }
                    }
                }
                udev_device_unref(dev);
            }
        }

        // Brief delay before checking again if no events
        if (ret == 0) {
            usleep(DETECTION_POLL_INTERVAL * 1000);
        }
    }

    // Clean up
    udev_monitor_unref(mon);
    udev_unref(udev);
}

/* Find the input event device for our HMI */
char* find_hmi_input_device(void) {
    struct udev *udev;
    struct udev_enumerate *enumerate;
    struct udev_list_entry *devices, *dev_list_entry;
    char *result = NULL;

    // Create udev context
    udev = udev_new();
    if (!udev) {
        return NULL;
    }

    // Create enumerate object for input devices
    enumerate = udev_enumerate_new(udev);
    udev_enumerate_add_match_subsystem(enumerate, "input");
    udev_enumerate_scan_devices(enumerate);

    // Get the list of input devices
    devices = udev_enumerate_get_list_entry(enumerate);

    // Iterate through input devices
    udev_list_entry_foreach(dev_list_entry, devices) {
        const char *path = udev_list_entry_get_name(dev_list_entry);
        struct udev_device *dev = udev_device_new_from_syspath(udev, path);

        if (dev) {
            const char *devnode = udev_device_get_devnode(dev);

            // Only interested in event devices
            if (devnode && strstr(devnode, "/dev/input/event") != NULL) {
                // Get the parent USB device
                struct udev_device *parent = udev_device_get_parent_with_subsystem_devtype(
                    dev, "usb", "usb_interface");

                if (parent) {
                    // Get the parent of the interface to reach the USB device itself
                    struct udev_device *usb_dev = udev_device_get_parent_with_subsystem_devtype(
                        parent, "usb", "usb_device");

                    if (usb_dev) {
                        const char *vendor = udev_device_get_sysattr_value(usb_dev, "idVendor");
                        const char *product = udev_device_get_sysattr_value(usb_dev, "idProduct");
                        const char *manufacturer = udev_device_get_sysattr_value(usb_dev, "manufacturer");
                        const char *product_name = udev_device_get_sysattr_value(usb_dev, "product");

                        // Check if this is our HMI device
                        if (vendor && product &&
                            strcmp(vendor, HMI_VENDOR_ID) == 0 &&
                            strcmp(product, HMI_PRODUCT_ID) == 0) {

                            if (manufacturer && product_name &&
                                strstr(manufacturer, HMI_MANUFACTURER) != NULL &&
                                strstr(product_name, HMI_PRODUCT_NAME) != NULL) {

                                // This is our device, save the event device path
                                result = strdup(devnode);
                                udev_device_unref(dev);
                                break;
                            }
                        }
                    }
                }
            }

            udev_device_unref(dev);
        }
    }

    // Clean up
    udev_enumerate_unref(enumerate);
    udev_unref(udev);

    return result;
}

/* Find the serial device for our HMI */
char* find_hmi_serial_device(void) {
    struct udev *udev;
    struct udev_enumerate *enumerate;
    struct udev_list_entry *devices, *dev_list_entry;
    char *result = NULL;

    // Create udev context
    udev = udev_new();
    if (!udev) {
        return NULL;
    }

    // Create enumerate object for tty devices
    enumerate = udev_enumerate_new(udev);
    udev_enumerate_add_match_subsystem(enumerate, "tty");
    udev_enumerate_scan_devices(enumerate);

    // Get the list of tty devices
    devices = udev_enumerate_get_list_entry(enumerate);

    // Iterate through tty devices
    udev_list_entry_foreach(dev_list_entry, devices) {
        const char *path = udev_list_entry_get_name(dev_list_entry);
        struct udev_device *dev = udev_device_new_from_syspath(udev, path);

        if (dev) {
            const char *devnode = udev_device_get_devnode(dev);

            // Only interested in ACM devices (likely to be our serial interface)
            if (devnode && strstr(devnode, "/dev/ttyACM") != NULL) {
                // Get the parent USB device
                struct udev_device *parent = udev_device_get_parent_with_subsystem_devtype(
                    dev, "usb", "usb_interface");

                if (parent) {
                    // Get the parent of the interface to reach the USB device itself
                    struct udev_device *usb_dev = udev_device_get_parent_with_subsystem_devtype(
                        parent, "usb", "usb_device");

                    if (usb_dev) {
                        const char *vendor = udev_device_get_sysattr_value(usb_dev, "idVendor");
                        const char *product = udev_device_get_sysattr_value(usb_dev, "idProduct");
                        const char *manufacturer = udev_device_get_sysattr_value(usb_dev, "manufacturer");
                        const char *product_name = udev_device_get_sysattr_value(usb_dev, "product");

                        // Check if this is our HMI device
                        if (vendor && product &&
                            strcmp(vendor, HMI_VENDOR_ID) == 0 &&
                            strcmp(product, HMI_PRODUCT_ID) == 0) {

                            if (manufacturer && product_name &&
                                strstr(manufacturer, HMI_MANUFACTURER) != NULL &&
                                strstr(product_name, HMI_PRODUCT_NAME) != NULL) {

                                // This is our device, save the tty device path
                                result = strdup(devnode);
                                udev_device_unref(dev);
                                break;
                            }
                        }
                    }
                }
            }

            udev_device_unref(dev);
        }
    }

    // Clean up
    udev_enumerate_unref(enumerate);
    udev_unref(udev);

    return result;
}

/* Device disconnection monitor thread function */
void *disconnection_monitor_thread(void *arg) {
    (void)arg; // Unused parameter

    struct udev *udev;
    struct udev_monitor *mon;
    int fd;

    // Create udev context
    udev = udev_new();
    if (!udev) {
        fprintf(stderr, "Failed to create udev context in monitor thread\n");
        return NULL;
    }

    // Set up monitoring
    mon = udev_monitor_new_from_netlink(udev, "udev");
    udev_monitor_filter_add_match_subsystem_devtype(mon, "usb", NULL);
    udev_monitor_enable_receiving(mon);
    fd = udev_monitor_get_fd(mon);

    // Set up polling
    struct pollfd fds[1];
    fds[0].fd = fd;
    fds[0].events = POLLIN;

    DEBUG_PRINT("Disconnection monitor thread started\n");

    // Track time for periodic checks
    time_t last_check_time = time(NULL);

    while (monitor_thread_running) {
        // Only check device connection periodically (every 5 seconds)
        time_t now = time(NULL);
        if (now - last_check_time >= 5) {
            // Check that the device is still present
            if (!check_device_connected()) {
                DEBUG_PRINT("Device disconnected (periodic check)\n");
                device_disconnected = true;
                break;
            }
            last_check_time = now;
        }

        // Poll for device events (focusing on removal events)
        int ret = poll(fds, 1, 1000); // 1 second timeout

        if (ret > 0 && (fds[0].revents & POLLIN)) {
            // Get the device
            struct udev_device *dev = udev_monitor_receive_device(mon);
            if (dev) {
                const char *action = udev_device_get_action(dev);

                // Only process remove events to avoid repeated detection
                if (action && strcmp(action, "remove") == 0) {
                    // Check if this is potentially our device
                    struct udev_device *usb_dev = udev_device_get_parent_with_subsystem_devtype(
                        dev, "usb", "usb_device");

                    if (usb_dev) {
                        const char *vendor = udev_device_get_sysattr_value(usb_dev, "idVendor");
                        const char *product = udev_device_get_sysattr_value(usb_dev, "idProduct");

                        if (vendor && product &&
                            strcmp(vendor, HMI_VENDOR_ID) == 0 &&
                            strcmp(product, HMI_PRODUCT_ID) == 0) {

                            DEBUG_PRINT("USB device disconnected (VID:PID %s:%s)\n", vendor, product);
                            device_disconnected = true;
                            udev_device_unref(dev);
                            break;
                        }
                    }
                }
                udev_device_unref(dev);
            }
        }
    }

    // Clean up
    udev_monitor_unref(mon);
    udev_unref(udev);

    DEBUG_PRINT("Disconnection monitor thread exiting\n");
    return NULL;
}

/* Check if the device is still connected (file handle check) */
bool check_device_connected(void) {
    // First, check if our file descriptors are still valid
    if (input_fd < 0 || serial_fd < 0) {
        return false;
    }

    // Try to check if the device nodes still exist
    struct stat input_stat, serial_stat;
    if (fstat(input_fd, &input_stat) < 0 || fstat(serial_fd, &serial_stat) < 0) {
        DEBUG_PRINT("Device file stats check failed\n");
        return false;
    }

    // IMPORTANT: Only check the device via udev occasionally, not on every call
    static time_t last_udev_check = 0;
    time_t now = time(NULL);

    // Only perform the expensive udev check once every 5 seconds
    if (now - last_udev_check > 5) {
        last_udev_check = now;

        // Check the actual devices via udev (but less frequently)
        bool device_present = check_device_present_silent();
        if (!device_present) {
            DEBUG_PRINT("Device not found in USB device list\n");
            return false;
        }
    }

    return true;
}
/* silent version of the device present check that doesn't print */
bool check_device_present_silent(void) {
    struct udev *udev;
    struct udev_enumerate *enumerate;
    struct udev_list_entry *devices, *dev_list_entry;
    bool found = false;

    // Create udev context
    udev = udev_new();
    if (!udev) {
        return false;
    }

    // Create enumerate object
    enumerate = udev_enumerate_new(udev);
    udev_enumerate_add_match_subsystem(enumerate, "usb");
    udev_enumerate_scan_devices(enumerate);

    // Get the list of matching devices
    devices = udev_enumerate_get_list_entry(enumerate);

    // Iterate through devices
    udev_list_entry_foreach(dev_list_entry, devices) {
        const char *path = udev_list_entry_get_name(dev_list_entry);
        struct udev_device *dev = udev_device_new_from_syspath(udev, path);

        if (dev) {
            const char *vendor = udev_device_get_sysattr_value(dev, "idVendor");
            const char *product = udev_device_get_sysattr_value(dev, "idProduct");

            // Check if this is our HMI device
            if (vendor && product &&
                strcmp(vendor, HMI_VENDOR_ID) == 0 &&
                strcmp(product, HMI_PRODUCT_ID) == 0) {

                found = true;
            }

            udev_device_unref(dev);

            if (found) break;
        }
    }

    // Clean up
    udev_enumerate_unref(enumerate);
    udev_unref(udev);

    return found;
}

// Function to set display power state
void set_display_power(bool power_on) {
    // Only send command if the state is changing
    if (power_on == display_powered_on) {
        return;
    }

    unsigned char cmd[2];
    cmd[0] = CMD_POWER_MODE;
    cmd[1] = power_on ? 0x01 : 0x00;

    // Send the power command through serial
    send_command(cmd, 2);

    display_powered_on = power_on;

    // Signal power save activation when turning off
    if (!power_on) {
        power_save_activated = true;
        DEBUG_PRINT("Power save activated - signaling all menus to exit\n");
    } else {
        power_save_activated = false;
    }

    DEBUG_PRINT("Display power set to: %s\n", power_on ? "ON" : "OFF");

    // If turning display back on, refresh the current menu
    if (power_on) {
        // Small delay to ensure display is ready
        usleep(DISPLAY_CMD_DELAY * 5);
        update_menu_display();
    }
}

// Function to update the activity timestamp
void update_activity_timestamp(void) {
    gettimeofday(&last_activity_time, NULL);

    // If the display was off, turn it back on
    if (power_save_enabled && !display_powered_on) {
        // Wake up display
        set_display_power(true);

        // Always return to main menu when waking up
        // Exit any submenu if we're in one
        // This would need custom handling in each submenu loop
        // or a global flag to signal menu exit
    }
}

// Function to check for power save timeout
void check_power_save_timeout(void) {
    if (!power_save_enabled || !display_powered_on) {
        return;
    }

    struct timeval now;
    gettimeofday(&now, NULL);

    // Calculate time diff in seconds
    long time_diff_sec = (now.tv_sec - last_activity_time.tv_sec);

    if (time_diff_sec >= POWER_SAVE_TIMEOUT_SEC) {
        DEBUG_PRINT("Power save timeout reached (%ld seconds of inactivity)\n", time_diff_sec);

        // Turn off the display
        set_display_power(false);
        power_save_activated = true;
    }
}
void check_power_save_and_signal_exit(void) {
    // Check for power save timeout
    if (power_save_enabled) {
        check_power_save_timeout();

        // If display turned off and we're in a submenu, signal it to exit
        if (!display_powered_on && current_submenu_function != NULL) {
            exit_current_submenu = true;
        }
    }
}
/* -----------------------------------------------------------------------------
 * Main function
 * -----------------------------------------------------------------------------*/

// Main program
int main(int argc, char *argv[]) {
    // Set signal handlers for clean exit
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    // Parse command line arguments
    bool args_provided = false;

    int opt;
    while ((opt = getopt(argc, argv, "i:s:vahp")) != -1) {
        switch (opt) {
            case 'i':
                config.input_device = optarg;
                args_provided = true;
                break;
            case 's':
                config.serial_device = optarg;
                args_provided = true;
                break;
            case 'v':
                config.verbose_mode = true;
                break;
            case 'a':
                // Auto-detect mode (will be used even if no args provided)
                printf("Auto-detection mode enabled\n");
                detect_and_run();
                return EXIT_SUCCESS;
        case 'p':
                power_save_enabled = true;
                printf("Power save mode enabled (timeout: %d seconds)\n", POWER_SAVE_TIMEOUT_SEC);
                break;
        case 'h':
                print_usage(argv[0]);
                return EXIT_SUCCESS;
            default:
                fprintf(stderr, "Try '%s -h' for more information.\n", argv[0]);
                return EXIT_FAILURE;
        }
    }

    // Check if no arguments were provided, show help in that case
    if (argc == 1 || !args_provided) {
        //print_usage(argv[0]);
        detect_and_run();
    return EXIT_SUCCESS;
    }

    printf("Using input device: %s\n", config.input_device);
    printf("Using serial device: %s\n", config.serial_device);
    if (config.verbose_mode) {
        printf("Verbose mode enabled\n");
    }

    // Open devices
    input_fd = open_input_device(config.input_device);
    if (input_fd < 0) {
        return EXIT_FAILURE;
    }

    serial_fd = open_serial_device(config.serial_device);
    if (serial_fd < 0) {
        close(input_fd);
        return EXIT_FAILURE;
    }

    // Initialize the menu
    init_menu();

    // Initial display with extra delay to make sure device is fully initialized
    usleep(STARTUP_DELAY); // 1s delay to give the device time to initialize
    printf("Initializing display...\n");

    // First clear the display
    send_clear();
    usleep(DISPLAY_CMD_DELAY * 15); // Very long delay after initial clear (150ms)

    // Draw a startup message (one line at a time with long delays)
    send_draw_text(0, 0, "Menu System");
    usleep(DISPLAY_CMD_DELAY * 10); // 100ms delay

    // Send one command at a time with long delays
    send_draw_text(0, 10, "Initializing...");
    usleep(DISPLAY_CMD_DELAY * 10); // 100ms delay

    // One more clear before showing the menu
    send_clear();
    usleep(DISPLAY_CMD_DELAY * 15); // 150ms delay

    // Now draw the menu
    update_menu_display();

    // Initialize timeval for key handling
    gettimeofday(&input_state.last_key_time, NULL);
    gettimeofday(&display_state.last_update_time, NULL);
    gettimeofday(&cmd_buffer.last_flush, NULL);

    // Main event loop
    struct timeval now;

    while (running) {
        // Prepare select
        fd_set readfds;
        struct timeval tv;
        FD_ZERO(&readfds);
        FD_SET(input_fd, &readfds);

        // Set a short timeout
        tv.tv_sec = 0;
        tv.tv_usec = INPUT_SELECT_TIMEOUT; // 20ms timeout

        // Wait for events with timeout
        int ret = select(input_fd + 1, &readfds, NULL, NULL, &tv);

        if (ret > 0 && FD_ISSET(input_fd, &readfds)) {
            handle_input_events();
        }

        // Process pending display updates if needed
        if (display_state.needs_update && !display_state.update_in_progress) {
            gettimeofday(&now, NULL);

            // Calculate time diff in milliseconds
            long time_diff_ms = get_time_diff_ms(&display_state.last_update_time, &now);

            // Only update if enough time has passed
            if (time_diff_ms > DISPLAY_UPDATE_DEBOUNCE) {
                update_menu_display();
            }
        }

        // Check if command buffer needs flushing
        gettimeofday(&now, NULL);
        long time_since_flush = get_time_diff_ms(&cmd_buffer.last_flush, &now);
        if (time_since_flush > CMD_BUFFER_FLUSH_INTERVAL && cmd_buffer.used > 0) {
            flush_cmd_buffer();
        }

    //check for powersave timeout
    check_power_save_timeout();

        // Small delay to reduce CPU usage
        usleep(MAIN_LOOP_DELAY); // 5ms
    }

    // Clean up
    send_clear();
    usleep(DISPLAY_CMD_DELAY);
    send_draw_text(0, 0, "Daemon stopped");

    cleanup_menu();

    if (input_fd >= 0) {
        close(input_fd);
    }

    if (serial_fd >= 0) {
        close(serial_fd);
    }

    pthread_mutex_destroy(&serial_mutex);

    return EXIT_SUCCESS;
}
