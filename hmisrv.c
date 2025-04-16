#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>  // Added for size_t
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
#include <linux/input.h>
#include <signal.h>
#include <pthread.h>
#include <sys/time.h>
#include <stdbool.h>  // Added for bool type

// Configurable paths to device files
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
#define CHAR_WIDTH 6   // Typical width of a character in pixels
#define CHAR_HEIGHT 8  // Default font height in pixels

// Menu configuration
#define MENU_START_Y 16    // Y position where menu items start
#define MENU_ITEM_SPACING 10 // Pixels between menu items

// Input event codes
#define EV_REL 0x02  // Relative movement event type
#define EV_KEY 0x01  // Key event type
#define REL_X  0x00  // X-axis code
#define BTN_LEFT 0x110  // Left mouse button code

// Protection against event flooding
#define MAX_EVENTS_PER_ITERATION 5

// Display command buffer delay (microseconds)
#define DISPLAY_CMD_DELAY 10000  // 10ms delay between display commands

// Program version
#define VERSION "1.0.0"

// Global variables
static int input_fd = -1;
static int serial_fd = -1;
static int running = 1;
static pthread_mutex_t serial_mutex = PTHREAD_MUTEX_INITIALIZER;
static bool verbose_mode = false;  // Flag for verbose output

// Helper macros for conditional debug output
#define DEBUG_PRINT(fmt, ...) \
    do { if (verbose_mode) fprintf(stderr, fmt, ##__VA_ARGS__); } while (0)

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

// For debouncing and preventing display flicker
static struct timeval last_display_update_time = {0, 0};
static int display_update_in_progress = 0;
static int display_needs_update = 0;

// For acceleration implementation
static struct timeval last_key_time = {0, 0};
static int consecutive_same_key = 0;
static int last_direction = 0; // 0=none, 1=left, 2=right

// Menu items
typedef struct {
    const char *label;
    void (*action)(void);
} MenuItem;

static int current_menu_item = 0;
static int menu_item_count = 0;
static MenuItem *menu_items = NULL;

// Function prototypes
void send_clear(void);
void send_draw_text(int x, int y, const char *text);
void send_set_cursor(int x, int y);
void send_invert(int invert);
void update_menu_display(void);
void handle_key_left(void);
void handle_key_right(void);
void handle_key_enter(void);
void action_brightness(void);
void update_brightness_display(int brightness);
void brightness_control_loop(void);

// Example menu action functions
void action_hello(void) {
    char text[64];
    
    // Format all text at once to prevent clearing between each line
    snprintf(text, sizeof(text), "Hello, World!\nPressed Enter!");
    
    // Clear first, then draw all text at once
    send_clear();
    usleep(DISPLAY_CMD_DELAY * 3);  // 30ms delay
    send_draw_text(0, 0, text);
    printf("Hello action selected\n");
    
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
    printf("Counter incremented to %d\n", counter);
    
    // Wait for user to see result before returning to menu
    sleep(2);
    update_menu_display();
}

void action_invert(void) {
    static int inverted = 0;
    inverted = !inverted;
    send_invert(inverted);
    printf("Display inversion toggled\n");
}

void action_exit(void) {
    running = 0;
    printf("Exiting application\n");
}

// Brightness control action function
void action_brightness(void) {
    int brightness = 128; // Start at 50% brightness
    
    // Initialize and enter the brightness control loop
    brightness_control_loop();
}

// Function to update the brightness display UI
void update_brightness_display(int brightness) {
    char text[64];
    int percentage = (brightness * 100) / 255;
    
    // Clear display
    send_clear();
    usleep(DISPLAY_CMD_DELAY * 3);
    
    // Draw title and percentage
    //send_draw_text(8, 0, "Brightness Control");
    send_draw_text(10, 0, "Brightness");
    
    snprintf(text, sizeof(text), "%d%%", percentage);
    send_draw_text(50, 15, text);
    
    // Draw progress bar (x, y, width, height, progress)
    // Use the progress percentage (0-100)
    unsigned char progress_cmd[6];
    progress_cmd[0] = CMD_PROGRESS_BAR;
    progress_cmd[1] = 10;    // x position
    progress_cmd[2] = 30;    // y position
    progress_cmd[3] = 108;   // width
    progress_cmd[4] = 15;    // height
    progress_cmd[5] = percentage; // progress (0-100)
    
    // Send the command to the display
    send_command(progress_cmd, 6);
    
    // Display instructions
    //send_draw_text(15, 50, "Rotate to adjust");
    
    // Also set the actual brightness
    unsigned char brightness_cmd[2];
    brightness_cmd[0] = CMD_BRIGHTNESS;
    brightness_cmd[1] = brightness;
    send_command(brightness_cmd, 2);
    
    DEBUG_PRINT("Brightness set to %d (%d%%)\n", brightness, percentage);
}

// The main brightness control loop
void brightness_control_loop(void) {
    int current_brightness = 128; // Start at 50%
    int running_brightness_menu = 1;
    int pending_update = 1;
    
    struct input_event ev;
    fd_set readfds;
    struct timeval tv;
    struct timeval last_event_time = {0, 0};
    struct timeval now;
    int paired_event_count = 0;
    int total_rel_x = 0;
    bool pending_movement = false;
    
    // Reset control variables
    consecutive_same_key = 0;
    last_direction = 0;
    
    DEBUG_PRINT("Entering brightness control mode\n");
    
    // Initial display
    update_brightness_display(current_brightness);
    
    while (running_brightness_menu && running) {
        // Prepare select
        FD_ZERO(&readfds);
        FD_SET(input_fd, &readfds);

        // Set a short timeout
        tv.tv_sec = 0;
        tv.tv_usec = 20000; // 20ms timeout

        // Wait for events with timeout
        int ret = select(input_fd + 1, &readfds, NULL, NULL, &tv);

        if (ret > 0 && FD_ISSET(input_fd, &readfds)) {
            int event_count = 0;
            int btn_press = 0;
            pending_movement = false;

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
                    if (last_event_time.tv_sec != 0) {
                        time_diff_ms = (now.tv_sec - last_event_time.tv_sec) * 1000 +
                                     (now.tv_usec - last_event_time.tv_usec) / 1000;
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
                    
                    DEBUG_PRINT("REL_X event: value=%d, total=%d, paired=%d, diff=%ldms\n", 
                           ev.value, total_rel_x, paired_event_count, time_diff_ms);
                    
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
            long time_since_last_ms = (now.tv_sec - last_event_time.tv_sec) * 1000 +
                                    (now.tv_usec - last_event_time.tv_usec) / 1000;
            
            if (pending_movement && (paired_event_count >= 2 || time_since_last_ms > 30)) {
                // Only process if we have a non-zero total
                if (total_rel_x != 0) {
                    // Left/Right changes brightness
                    if (total_rel_x < 0) {
                        // Decrease brightness (rotate left)
                        current_brightness -= 10;
                        if (current_brightness < 0) current_brightness = 0;
                        pending_update = 1;
                    } else if (total_rel_x > 0) {
                        // Increase brightness (rotate right)
                        current_brightness += 10;
                        if (current_brightness > 255) current_brightness = 255;
                        pending_update = 1;
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
        }
        
        // Update the display if needed
        if (pending_update) {
            update_brightness_display(current_brightness);
            pending_update = 0;
        }

        // Small delay to reduce CPU usage
        usleep(5000); // 5ms
    }
    
    // When exiting the brightness menu, update the main menu
    update_menu_display();
}

// Initialize the menu items
void init_menu(void) {
    menu_item_count = 5;  // Updated count to include the brightness option
    menu_items = (MenuItem *)malloc(menu_item_count * sizeof(MenuItem));

    menu_items[0].label = "Hello World";
    menu_items[0].action = action_hello;

    menu_items[1].label = "Counter";
    menu_items[1].action = action_counter;

    menu_items[2].label = "Invert Display";
    menu_items[2].action = action_invert;

    menu_items[3].label = "Brightness";  // New brightness menu item
    menu_items[3].action = action_brightness;

    menu_items[4].label = "Exit";  // Exit is now item 4 instead of 3
    menu_items[4].action = action_exit;
}

// Clean up menu memory
void cleanup_menu(void) {
    free(menu_items);
    menu_items = NULL;
    menu_item_count = 0;
}

// Update just the lines that need to change when selection changes
void update_selection(int old_selection, int new_selection) {
    // Only update the two lines that have changed
    if (old_selection >= 0 && old_selection < menu_item_count) {
        char buffer[32];
        // Update the old selection (remove the arrow)
        snprintf(buffer, sizeof(buffer), "  %s", menu_items[old_selection].label);
        // Calculate y position based on menu start position and spacing
        int y_pos = MENU_START_Y + (old_selection * MENU_ITEM_SPACING);
        send_draw_text(0, y_pos, buffer);
        usleep(DISPLAY_CMD_DELAY);
        DEBUG_PRINT("Updating line %d: '%s' at y=%d\n", old_selection + 1, buffer, y_pos);
    }
    
    if (new_selection >= 0 && new_selection < menu_item_count) {
        char buffer[32];
        // Update the new selection (add the arrow)
        snprintf(buffer, sizeof(buffer), "> %s", menu_items[new_selection].label);
        // Calculate y position based on menu start position and spacing
        int y_pos = MENU_START_Y + (new_selection * MENU_ITEM_SPACING);
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
    long time_diff_ms = (now.tv_sec - last_display_update_time.tv_sec) * 1000 +
                        (now.tv_usec - last_display_update_time.tv_usec) / 1000;
    
    // Debounce display updates (don't update more often than every 100ms)
    if (time_diff_ms < 100) {
        display_needs_update = 1;
        return;
    }
    
    // If an update is already in progress, just mark that we need another one
    if (display_update_in_progress) {
        display_needs_update = 1;
        return;
    }
    
    display_update_in_progress = 1;
    display_needs_update = 0;
    
    DEBUG_PRINT("Updating display with menu item %d of %d\n", current_menu_item + 1, menu_item_count);
    
    // Clear the display first
    send_clear();
    usleep(DISPLAY_CMD_DELAY * 5);  // 50ms delay after clear
    
    // Draw a title at the top
    send_draw_text(24, 0, "MAIN MENU");
    usleep(DISPLAY_CMD_DELAY);
    
    // Draw a separator line
    send_draw_text(0, 8, "----------------");
    usleep(DISPLAY_CMD_DELAY);
    
    // Now draw each menu item with proper spacing
    for (int i = 0; i < menu_item_count; i++) {
        char buffer[32];
        
        // Format menu item text with selection indicator
        if (i == current_menu_item) {
            snprintf(buffer, sizeof(buffer), "> %s", menu_items[i].label);
        } else {
            snprintf(buffer, sizeof(buffer), "  %s", menu_items[i].label);
        }
        
        // Calculate y position based on menu start position and spacing
        int y_pos = MENU_START_Y + (i * MENU_ITEM_SPACING);
        send_draw_text(0, y_pos, buffer);
        
        // Print debug info
        DEBUG_PRINT("Menu item %d: '%s' at y=%d\n", i+1, buffer, y_pos);
        
        // Add delay between drawing commands
        usleep(DISPLAY_CMD_DELAY);
    }
    
    // Make sure all commands are processed
    usleep(DISPLAY_CMD_DELAY * 2);
    
    // Update the timestamp
    gettimeofday(&last_display_update_time, NULL);
    
    display_update_in_progress = 0;
    
    // If another update was requested during this one, do it now
    if (display_needs_update) {
        usleep(DISPLAY_CMD_DELAY * 2);
        display_needs_update = 0;
        update_menu_display();
    }
}

// Handle key press events with acceleration
void handle_key_left(void) {
    struct timeval now;
    gettimeofday(&now, NULL);

    // Calculate time diff in milliseconds
    long time_diff_ms = (now.tv_sec - last_key_time.tv_sec) * 1000 +
                        (now.tv_usec - last_key_time.tv_usec) / 1000;

    // Check if this is a rapid succession of the same key
    if (last_direction == 1 && time_diff_ms < 200) { // 200ms
        consecutive_same_key++;
    } else {
        consecutive_same_key = 0;
    }

    // Apply acceleration (move more items for fast rotation)
    int steps = 1;
    if (consecutive_same_key > 5) {
        steps = 3; // Move 3 items at a time when rotating quickly
    } else if (consecutive_same_key > 2) {
        steps = 2; // Move 2 items at a time
    }

    // Remember old selection
    int old_selection = current_menu_item;

    // Move the selection up
    for (int i = 0; i < steps; i++) {
        if (current_menu_item > 0) {
            current_menu_item--;
        }
    }

    // If the selection actually changed, update the display
    if (old_selection != current_menu_item) {
        update_selection(old_selection, current_menu_item);
    }

    last_direction = 1;
    last_key_time = now;
    DEBUG_PRINT("LEFT key pressed (steps: %d)\n", steps);
}

void handle_key_right(void) {
    struct timeval now;
    gettimeofday(&now, NULL);

    // Calculate time diff in milliseconds
    long time_diff_ms = (now.tv_sec - last_key_time.tv_sec) * 1000 +
                        (now.tv_usec - last_key_time.tv_usec) / 1000;

    // Check if this is a rapid succession of the same key
    if (last_direction == 2 && time_diff_ms < 200) { // 200ms
        consecutive_same_key++;
    } else {
        consecutive_same_key = 0;
    }

    // Apply acceleration (move more items for fast rotation)
    int steps = 1;
    if (consecutive_same_key > 5) {
        steps = 3; // Move 3 items at a time when rotating quickly
    } else if (consecutive_same_key > 2) {
        steps = 2; // Move 2 items at a time
    }

    // Remember old selection
    int old_selection = current_menu_item;

    // Move the selection down
    for (int i = 0; i < steps; i++) {
        if (current_menu_item < menu_item_count - 1) {
            current_menu_item++;
        }
    }

    // If the selection actually changed, update the display
    if (old_selection != current_menu_item) {
        update_selection(old_selection, current_menu_item);
    }

    last_direction = 2;
    last_key_time = now;
    DEBUG_PRINT("RIGHT key pressed (steps: %d)\n", steps);
}

void handle_key_enter(void) {
    DEBUG_PRINT("ENTER key pressed\n");
    if (current_menu_item >= 0 && current_menu_item < menu_item_count) {
        if (menu_items[current_menu_item].action) {
            menu_items[current_menu_item].action();
        }
    }
}

// Serial communication functions with proper flushing
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

void send_clear(void) {
    unsigned char cmd = CMD_CLEAR;
    send_command(&cmd, 1);
}

void send_draw_text(int x, int y, const char *text) {
    size_t text_len = strlen(text);
    unsigned char *cmd = (unsigned char *)malloc(text_len + 3);

    cmd[0] = CMD_DRAW_TEXT;
    cmd[1] = (unsigned char)x;
    cmd[2] = (unsigned char)y;
    memcpy(cmd + 3, text, text_len);

    send_command(cmd, text_len + 3);
    free(cmd);
}

void send_set_cursor(int x, int y) {
    unsigned char cmd[3];
    cmd[0] = CMD_SET_CURSOR;
    cmd[1] = (unsigned char)x;
    cmd[2] = (unsigned char)y;
    send_command(cmd, 3);
}

void send_invert(int invert) {
    unsigned char cmd[2];
    cmd[0] = CMD_INVERT;
    cmd[1] = invert ? 1 : 0;
    send_command(cmd, 2);
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

// Main program
int main(int argc, char *argv[]) {
    // Set signal handlers for clean exit
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    // Parse command line arguments
    const char *input_device = DEFAULT_INPUT_DEVICE;
    const char *serial_device = DEFAULT_SERIAL_DEVICE;
    bool args_provided = false;

    int opt;
    while ((opt = getopt(argc, argv, "i:s:vh")) != -1) {
        switch (opt) {
            case 'i':
                input_device = optarg;
                args_provided = true;
                break;
            case 's':
                serial_device = optarg;
                args_provided = true;
                break;
            case 'v':
                verbose_mode = true;
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

    printf("Using input device: %s\n", input_device);
    printf("Using serial device: %s\n", serial_device);
    if (verbose_mode) {
        printf("Verbose mode enabled\n");
    }

    // Open devices
    input_fd = open_input_device(input_device);
    if (input_fd < 0) {
        return EXIT_FAILURE;
    }

    serial_fd = open_serial_device(serial_device);
    if (serial_fd < 0) {
        close(input_fd);
        return EXIT_FAILURE;
    }

    // Initialize the menu
    init_menu();

    // Initial display with extra delay to make sure device is fully initialized
    sleep(1); // Give the device time to initialize
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
    gettimeofday(&last_key_time, NULL);
    gettimeofday(&last_display_update_time, NULL);

    // Main event loop
    struct input_event ev;
    fd_set readfds;
    struct timeval tv;
    struct timeval last_event_time = {0, 0};
    int paired_event_count = 0;
    int total_rel_x = 0;

while (running) {
        // Prepare select
        FD_ZERO(&readfds);
        FD_SET(input_fd, &readfds);

        // Set a short timeout
        tv.tv_sec = 0;
        tv.tv_usec = 20000; // 20ms timeout

        // Wait for events with timeout
        int ret = select(input_fd + 1, &readfds, NULL, NULL, &tv);

        if (ret > 0 && FD_ISSET(input_fd, &readfds)) {
            struct timeval now;
            int event_count = 0;
            int btn_press = 0;
            bool pending_movement = false;

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
                    if (last_event_time.tv_sec != 0) {
                        time_diff_ms = (now.tv_sec - last_event_time.tv_sec) * 1000 +
                                     (now.tv_usec - last_event_time.tv_usec) / 1000;
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
                    
                    DEBUG_PRINT("REL_X event: value=%d, total=%d, paired=%d, diff=%ldms\n", 
                           ev.value, total_rel_x, paired_event_count, time_diff_ms);
                    
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
            long time_since_last_ms = (now.tv_sec - last_event_time.tv_sec) * 1000 +
                                    (now.tv_usec - last_event_time.tv_usec) / 1000;
            
            if (pending_movement && (paired_event_count >= 2 || time_since_last_ms > 30)) {
                // Only process if we have a non-zero total
                if (total_rel_x != 0) {
                    // Process movement based on total relative movement
                    if (total_rel_x < 0) {
                        handle_key_left();
                    } else if (total_rel_x > 0) {
                        handle_key_right();
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
        }

        // Process pending display updates if needed
        if (display_needs_update && !display_update_in_progress) {
            struct timeval now;
            gettimeofday(&now, NULL);
            
            // Calculate time diff in milliseconds
            long time_diff_ms = (now.tv_sec - last_display_update_time.tv_sec) * 1000 +
                              (now.tv_usec - last_display_update_time.tv_usec) / 1000;
            
            // Only update if enough time has passed
            if (time_diff_ms > 100) {
                update_menu_display();
            }
        }

        // Small delay to reduce CPU usage
        usleep(5000); // 5ms
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
