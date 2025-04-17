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
/* -----------------------------------------------------------------------------
 * Configuration definitions
 * -----------------------------------------------------------------------------*/

// Device paths
#define DEFAULT_INPUT_DEVICE "/dev/input/event0"
#define DEFAULT_SERIAL_DEVICE "/dev/ttyACM0"

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

// Input event handling limits
#define MAX_EVENTS_PER_ITERATION 5
#define MAX_ACCELERATION_STEPS 3

// Serial command buffer
#define CMD_BUFFER_SIZE 256

// Program version
#define VERSION "1.2.0"

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

// Menu data
static int menu_item_count = 0;
static MenuItem *menu_items = NULL;

/* -----------------------------------------------------------------------------
 * Helper macros and function prototypes
 * -----------------------------------------------------------------------------*/

// Helper macros for conditional debug output
#define DEBUG_PRINT(fmt, ...) \
    do { if (config.verbose_mode) fprintf(stderr, fmt, ##__VA_ARGS__); } while (0)

// Function prototypes
// Menu functions
void init_menu(void);
void cleanup_menu(void);
void update_menu_display(void);
void update_selection(int old_selection, int new_selection);

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
void get_network_info(char *ip_str, size_t ip_len, char *mac_str, size_t mac_len);
//void get_system_info(char *cpu_str, size_t cpu_len, char *mem_str, size_t mem_len);
void get_system_info(char *cpu_str, size_t cpu_len, char *mem_total_str, size_t mem_total_len,
                    char *mem_free_str, size_t mem_free_len);//)
void get_system_info_with_percent(char *cpu_str, size_t cpu_len, char *mem_total_str, 
                               size_t mem_total_len, char *mem_free_str, size_t mem_free_len,
                               int *cpu_percentage, int *mem_percentage);

// Menu actions
void action_hello(void);
void action_counter(void);
void action_invert(void);
void action_exit(void);

// Utility functions
void print_usage(const char *program_name);
void handle_signal(int sig);
void set_nonblocking(int fd);
int open_input_device(const char *device_path);
int open_serial_device(const char *device_path);
long get_time_diff_ms(struct timeval *start, struct timeval *end);

/* -----------------------------------------------------------------------------
 * Menu action functions
 * -----------------------------------------------------------------------------*/

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
        } else if ((size_t)bytes_written < cmd_buffer.used) {
            fprintf(stderr, "Warning: Only wrote %zd of %zu bytes\n",
                   bytes_written, cmd_buffer.used);
        }
        // Flush the output
        tcdrain(serial_fd);
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
        } else if ((size_t)bytes_written < length) {
            fprintf(stderr, "Warning: Only wrote %zd of %zu bytes\n", bytes_written, length);
        }

        // Flush the output buffer to ensure command is sent immediately
        tcdrain(serial_fd);
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
    menu_item_count = 7;  // Increased from 5 to 7
    menu_items = (MenuItem *)malloc(menu_item_count * sizeof(MenuItem));
    if (!menu_items) {
        fprintf(stderr, "Memory allocation failed in init_menu()\n");
        exit(EXIT_FAILURE);
    }

    menu_items[0].label = "Hello World";
    menu_items[0].action = action_hello;

    menu_items[1].label = "Counter";
    menu_items[1].action = action_counter;

    menu_items[2].label = "Invert Display";
    menu_items[2].action = action_invert;

    menu_items[3].label = "Brightness";
    menu_items[3].action = action_brightness;

    // New menu items
    menu_items[4].label = "Network Settings";
    menu_items[4].action = action_network_settings;

    menu_items[5].label = "System Stats";
    menu_items[5].action = action_system_stats;

    // Exit is now item 6 instead of 4
    menu_items[6].label = "Exit";
    menu_items[6].action = action_exit;
}

// Clean up menu memory
void cleanup_menu(void) {
    free(menu_items);
    menu_items = NULL;
    menu_item_count = 0;
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

void handle_key_enter(void) {
    DEBUG_PRINT("ENTER key pressed\n");
    if (display_state.current_menu_item >= 0 && display_state.current_menu_item < menu_item_count) {
        if (menu_items[display_state.current_menu_item].action) {
            menu_items[display_state.current_menu_item].action();
        }
    }
}

// Handle all input events
void handle_input_events(void) {
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
                DEBUG_PRINT("Button pressed, exiting brightness control\n");
                running_brightness_menu = 0;
            }

            // Now handle the movement for brightness control
            gettimeofday(&now, NULL);
            long time_since_last_ms = get_time_diff_ms(&input_state.last_event_time, &now);

            if (pending_movement && (input_state.paired_event_count >= 2 || time_since_last_ms > EVENT_PROCESS_THRESHOLD)) {
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
    
    // Get network information
    get_network_info(ip_str, sizeof(ip_str), mac_str, sizeof(mac_str));
    
    // Clear display and show network info
    send_clear();
    usleep(DISPLAY_CMD_DELAY * 3);
    
    // Draw the title
    send_draw_text(0, 0, "Network Settings");
    usleep(DISPLAY_CMD_DELAY);
    
    // Draw a separator
    send_draw_text(0, 8, "----------------");
    usleep(DISPLAY_CMD_DELAY);
    
    // Format IP with line breaks - first line shows label
    send_draw_text(0, 16, "IP:");
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

void get_network_info(char *ip_str, size_t ip_len, char *mac_str, size_t mac_len) {
    struct ifaddrs *ifaddr, *ifa;
    int family, s;
    char host[NI_MAXHOST];
    bool found_ip = false;

    // Initialize with defaults
    snprintf(ip_str, ip_len, "Not connected");
    snprintf(mac_str, mac_len, "Not available");

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

        snprintf(ip_str, ip_len, "%s (%s)", host, ifa->ifa_name);
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

                snprintf(ip_str, ip_len, "%s (lo)", host);

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
    send_draw_text(0, 0, "System Stats");
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
/*void action_system_stats(void) {
    char cpu_str[16] = "Unknown";
    char mem_total_str[32] = "Unknown";
    char mem_free_str[32] = "Unknown";
    
    // Clear display and show initial system info
    send_clear();
    usleep(DISPLAY_CMD_DELAY * 3);
    send_draw_text(0, 0, "System Stats");
    usleep(DISPLAY_CMD_DELAY);
    
    // Draw a separator
    send_draw_text(0, 8, "----------------");
    usleep(DISPLAY_CMD_DELAY);
    
    int running_stats_menu = 1;
    struct timeval last_update = {0, 0};
    
    DEBUG_PRINT("Displaying system stats...\n");
    
    while (running_stats_menu && running) {
        // Update stats every STAT_UPDATE_SEC seconds
        struct timeval now;
        gettimeofday(&now, NULL);
        long time_diff_sec = (now.tv_sec - last_update.tv_sec);
        
        if (time_diff_sec >= STAT_UPDATE_SEC || last_update.tv_sec == 0) {
            // Get updated system information
            get_system_info(cpu_str, sizeof(cpu_str), mem_total_str, sizeof(mem_total_str), 
                           mem_free_str, sizeof(mem_free_str));
            
        if (last_update.tv_sec == 0) {
               send_draw_text(0, 16, "CPU-Load:");
               send_draw_text(0, 26, "Mem:");
               send_draw_text(0, 36, "Free:");
          }
    
          // Clear the value areas before writing new values
          send_draw_text(70, 16, "        "); // Clear CPU value area
          send_draw_text(50, 26, "               "); // Clear Memory value area
          send_draw_text(50, 36, "               "); // Clear Free value area
    
           // Small delay to ensure clear takes effect
           usleep(DISPLAY_CMD_DELAY);
    
            // Now update with new values
            send_draw_text(70, 16, cpu_str);
            send_draw_text(50, 26, mem_total_str);
            send_draw_text(50, 36, mem_free_str);            
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
*/

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
    while ((opt = getopt(argc, argv, "i:s:vh")) != -1) {
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
        print_usage(argv[0]);
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
