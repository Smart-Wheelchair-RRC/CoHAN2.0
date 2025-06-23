// To build: gcc sim.c -o sim -lSDL2 -lm -lyaml
// gcc -shared -fPIC -o ../lib/libsim.so sim.c -lSDL2 -lm -lyaml
#include <SDL2/SDL.h>
#include <X11/Xlib.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <yaml.h>

#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif

#define ROBOT_STATE_SIZE 6
#define FPS 60
#define COLOR_WHITE 255, 255, 255, 255
#define COLOR_BLACK 0, 0, 0, 255
#define MAX_ENTITIES 100
#define MAX_LASER_POINTS 1081    // Maximum number of laser points
#define VELOCITY_TIMEOUT_MS 100  // Time in milliseconds before zeroing velocity
#define COLOR_ALPHA_OPAQUE 255
#define CIRCLE_BASE_SEGMENTS 32
#define CIRCLE_SEGMENTS_PER_RADIUS 1.5
#define TWO_PI 6.28318530717958647692
#define PI 3.14159265358979323846
#define DEG_360 360
#define DEG_30 30
#define DEG_180 180.0
#define LASER_DEFAULT_RANGE 10.0
#define ZOOM_STEP 1.1
#define MS_PER_SECOND 1000

// SIGINT handler flag and function declaration
static volatile sig_atomic_t sigint_received = 0;
static void handle_sigint(int sig);

// --- Zoom and Pan globals and helper ---
static double zoom = 1.0;
static double pan_x = 0.0, pan_y = 0.0;
static int dragging = 0;
static int drag_start_x = 0, drag_start_y = 0;
static double pan_start_x = 0.0, pan_start_y = 0.0;

// Global variable for render
bool RENDER_SIM = true;

// Convert map pixel to screen pixel
static void map_to_screen(int mx, int my, int* sx, int* sy) {
  *sx = (int)((zoom * (double)mx) + pan_x);
  *sy = (int)((zoom * (double)my) + pan_y);
}

/**
 * @brief Structure representing a mobile entity (robot or human) in the simulation
 * @details Contains position, orientation, velocity, and appearance properties
 */
typedef struct {
  double x, y, theta;    // Floating-point positions and orientation
  double radius;         // Collision radius of the entity
  SDL_Color color;       // Color for rendering
  double vx, vy, omega;  // Linear and angular velocities
} Entity;

typedef struct {
  char image[256];
  double resolution;
  double origin[3];
  int negate;
  double occupied_thresh;
  double free_thresh;
  double speed;          // Keyboard linear speed
  double angular_speed;  // Keyboard angular speed
} MapYamlConfig;

/**
 * @brief Simulation entity wrapper with control and movement options
 */
typedef struct {
  Entity entity;
  double initial_x;
  double initial_y;
  double initial_theta;
  bool use_keyboard;
  bool use_differential;
  bool use_laser;
  double laser_range;
  int laser_resolution;
  double laser_angle;
  float* laser_data;         // Dynamically allocated array for laser data
  Uint32 last_vel_command;   // Timestamp of last velocity command
  double head_rotation;      // Rotation for inner circle/dots (only for keyboard entity)
  double target_head_angle;  // Target angle for smooth rotation
  bool head_at_target;
  // Optionally: add more per-entity config here
} SimEntity;

typedef struct {
  double vel_x;
  double vel_y;
} Pair;

static SimEntity g_entities[MAX_ENTITIES];
static int g_entity_count = 0;
static bool keyboard_in_use = false;
static void reset_all_entities();

/**
 * @brief Draws a visually correct circle outline at any zoom using parametric points
 * @param renderer SDL renderer context
 * @param cx Center x-coordinate (screen px)
 * @param cy Center y-coordinate (screen px)
 * @param radius Circle radius in pixels (screen px)
 * @param color SDL_Color for the circle
 */
static void draw_circle(SDL_Renderer* renderer, int cx, int cy, int radius, SDL_Color color) {
  SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, COLOR_ALPHA_OPAQUE);
  if (radius < 2) {
    SDL_RenderDrawPointF(renderer, cx, cy);
    return;
  }
  int segments = CIRCLE_BASE_SEGMENTS + (int)(radius * CIRCLE_SEGMENTS_PER_RADIUS);
  double theta_step = TWO_PI / segments;
  int prev_x = cx + radius;
  int prev_y = cy;
  for (int i = 1; i <= segments; ++i) {
    double theta = i * theta_step;
    int x = cx + (int)round(radius * cos(theta));
    int y = cy + (int)round(radius * sin(theta));
    SDL_RenderDrawLineF(renderer, prev_x, prev_y, x, y);
    prev_x = x;
    prev_y = y;
  }
}

/**
 * @brief Converts world coordinates (meters) to map pixel coordinates
 * @param wx X coordinate in world frame (meters)
 * @param wy Y coordinate in world frame (meters)
 * @param mx Output map x-coordinate (pixels)
 * @param my Output map y-coordinate (pixels)
 * @param config Map configuration parameters
 * @param map_h Map height in pixels
 */
static void world_to_map_coords(double wx, double wy, int* mx, int* my, MapYamlConfig* config, int map_h) {
  *mx = (int)round((wx - config->origin[0]) / config->resolution);
  *my = map_h - 1 - (int)round((wy - config->origin[1]) / config->resolution);
}

/**
 * @brief Converts map pixel coordinates to world coordinates (meters)
 * @param mx Map x-coordinate (pixels)
 * @param my Map y-coordinate (pixels)
 * @param wx Output x coordinate in world frame (meters)
 * @param wy Output y coordinate in world frame (meters)
 * @param config Map configuration parameters
 * @param map_h Map height in pixels
 */
static void map_to_world_coords(int mx, int my, double* wx, double* wy, MapYamlConfig* config, int map_h) {
  *wx = mx * config->resolution + config->origin[0];
  *wy = (map_h - my) * config->resolution + config->origin[1];
}

/**
 * @brief Renders an entity (robot or human) to the SDL window
 * @param renderer SDL renderer context
 * @param e Entity to draw
 * @param config Map configuration parameters
 * @param map_h Map height in pixels
 */
static void draw_entity(SDL_Renderer* renderer, Entity* e, MapYamlConfig* config, int map_h, double head_rotation) {
  int px;
  int py;
  world_to_map_coords(e->x, e->y, &px, &py, config, map_h);
  int sx;
  int sy;
  map_to_screen(px, py, &sx, &sy);
  int radius_px = (int)(e->radius / config->resolution * zoom);
  draw_circle(renderer, sx, sy, radius_px, e->color);
  // Draw small inner circle
  int small_radius = (int)(radius_px * 0.35);
  draw_circle(renderer, sx, sy, small_radius, e->color);
  // Draw two dots inside the small circle, facing heading + head_rotation
  double heading_angle = -e->theta + head_rotation;
  double dot_dist = small_radius * 0.6;
  double dot_radius = fmax(2, small_radius * 0.18);
  for (int i = -1; i <= 1; i += 2) {
    double dot_angle = heading_angle + (i * 0.3);
    int dot_x = sx + (int)(dot_dist * cos(dot_angle));
    int dot_y = sy + (int)(dot_dist * sin(dot_angle));
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, COLOR_ALPHA_OPAQUE);
    for (int a = 0; a < 360; a += 20) {
      double th = a * PI / 180.0;
      int px = dot_x + (int)(dot_radius * cos(th));
      int py = dot_y + (int)(dot_radius * sin(th));
      SDL_RenderDrawPointF(renderer, px, py);
    }
  }
  // Draw heading arrow (should not rotate with inner circle)
  int arrow_len = radius_px * 2;
  double arrow_angle = -e->theta;  // Only use entity heading, not head_rotation
  int ax = sx + (int)(arrow_len * cos(arrow_angle));
  int ay = sy + (int)(arrow_len * sin(arrow_angle));
  SDL_SetRenderDrawColor(renderer, e->color.r, e->color.g, e->color.b, COLOR_ALPHA_OPAQUE);
  SDL_RenderDrawLineF(renderer, sx, sy, ax, ay);
}

static void skip_pgm_comments(FILE* f) {
  int c;
  while ((c = fgetc(f)) != EOF) {
    if (c == '#') {
      while ((c = fgetc(f)) != '\n' && c != EOF) {
      }
    } else if (!isspace(c)) {
      ungetc(c, f);
      break;
    }
  }
}

static bool** load_occupancy_grid_pgm(const char* filename, int* w, int* h) {
  FILE* f = fopen(filename, "rb");
  if (!f) {
    printf("Failed to open PGM file: %s\n", filename);
    return NULL;
  }
  char header[3];
  if (fscanf(f, "%2s", header) != 1 || (strcmp(header, "P5") != 0 && strcmp(header, "P2") != 0)) {
    printf("Unsupported PGM format (must be P2 or P5)\n");
    fclose(f);
    return NULL;
  }
  skip_pgm_comments(f);
  if (fscanf(f, "%d", w) != 1) {
    printf("Invalid PGM width\n");
    fclose(f);
    return NULL;
  }
  skip_pgm_comments(f);
  if (fscanf(f, "%d", h) != 1) {
    printf("Invalid PGM height\n");
    fclose(f);
    return NULL;
  }
  skip_pgm_comments(f);
  int maxval;
  if (fscanf(f, "%d", &maxval) != 1) {
    printf("Invalid PGM maxval\n");
    fclose(f);
    return NULL;
  }
  fgetc(f);  // skip single whitespace after header
  bool** grid = (bool**)malloc(*w * sizeof(bool*));
  for (int x = 0; x < *w; x++) {
    grid[x] = (bool*)malloc(*h * sizeof(bool));
  }
  if (strcmp(header, "P5") == 0) {
    // Binary
    for (int y = 0; y < *h; y++) {
      for (int x = 0; x < *w; x++) {
        int val = fgetc(f);
        grid[x][y] = (val < maxval / 2);  // black = occupied
      }
    }
  } else {
    // ASCII
    for (int y = 0; y < *h; y++) {
      for (int x = 0; x < *w; x++) {
        int val;
        int result = fscanf(f, "%d", &val);
        if (result != 1) {
            fprintf(stderr, "Error reading value from PGM file\n");
            return NULL;
        }
        grid[x][y] = (val < maxval / 2);
      }
    }
  }
  fclose(f);
  return grid;
}

/**
 * @brief Updates the entity's vx, vy from local (body) frame to world frame using current heading
 * @param e Entity whose velocity to update
 */
Pair get_world_velocity(Entity* e) {
  double cos_t = cos(e->theta);
  double sin_t = sin(e->theta);
  // Store local velocities temporarily
  double vx_local = e->vx;
  double vy_local = e->vy;
  // Correct transformation from local to world frame
  double vx_world = (vx_local * cos_t) - (vy_local * sin_t);
  double vy_world = (vx_local * sin_t) + (vy_local * cos_t);
  // Update entity velocities with world frame values
  Pair velocity;
  velocity.vel_x = vx_world;
  velocity.vel_y = vy_world;
  return velocity;
}

static bool can_move(Entity* e, double x, double y, bool** grid, int map_w, int map_h, MapYamlConfig* config) {
  int px;
  int py;
  world_to_map_coords(x, y, &px, &py, config, map_h);
  int r_px = (int)(e->radius / config->resolution);
  if (px < r_px || px >= map_w - r_px || py < r_px || py >= map_h - r_px) {
    return false;
  }
  for (int angle = 0; angle < DEG_360; angle += DEG_30) {
    double rad = angle * PI / DEG_180;
    int cx = px + (int)(r_px * cos(rad));
    int cy = py + (int)(r_px * sin(rad));
    if (cx < 0 || cx >= map_w || cy < 0 || cy >= map_h) {
      return false;
    }
    if (grid[cx][cy]) {
      return false;
    }
  }
  return true;
}

/**
 * @brief Updates entity position with collision checking
 * @param e Entity to move
 * @param dt Time step in seconds
 * @param grid Occupancy grid
 * @param map_w Map width in pixels
 * @param map_h Map height in pixels
 * @param config Map configuration parameters
 */
void move_entity_holonomic(Entity* e, double dt, bool** grid, int map_w, int map_h, MapYamlConfig* config) {
  Pair world_vel = get_world_velocity(e);
  double new_x = e->x + (world_vel.vel_x * dt);
  double new_y = e->y + (world_vel.vel_y * dt);
  if (can_move(e, new_x, new_y, grid, map_w, map_h, config)) {
    e->x = new_x;
    e->y = new_y;
    e->theta += e->omega * dt;
    if (e->theta > 2 * M_PI) {
      e->theta -= 2 * M_PI;
    }
    if (e->theta < 0) {
      e->theta += 2 * M_PI;
    }
  } else {
    e->vx = 0;
    e->vy = 0;
    e->omega = 0;
  }
}

/**
 * @brief Updates entity position using differential drive kinematics, computing v from vx, vy, and heading
 * @param e Entity to move (uses e->vx, e->vy, e->omega)
 * @param dt Time step (s)
 * @param grid Occupancy grid
 * @param map_w Map width in pixels
 * @param map_h Map height in pixels
 * @param config Map configuration parameters
 */
void move_entity_differential(Entity* e, double dt, bool** grid, int map_w, int map_h, MapYamlConfig* config) {
  double v = e->vx;  // Forward velocity in robot's frame
  double omega = e->omega;
  double theta = e->theta;
  double new_x, new_y, new_theta;
  // Threshold for considering omega as zero
  const double EPSILON = 1e-6;
  if (fabs(omega) < EPSILON) {
    // Straight motion
    new_x = e->x + v * cos(theta) * dt;
    new_y = e->y + v * sin(theta) * dt;
    new_theta = theta;
  } else {
    // Arc motion
    double r = v / omega;
    new_theta = theta + omega * dt;
    new_x = e->x + r * (sin(new_theta) - sin(theta));
    new_y = e->y - r * (cos(new_theta) - cos(theta));
  }
  // Normalize new_theta to [0, 2PI)
  if (new_theta >= TWO_PI) new_theta -= TWO_PI;
  if (new_theta < 0) new_theta += TWO_PI;
  if (can_move(e, new_x, new_y, grid, map_w, map_h, config)) {
    e->x = new_x;
    e->y = new_y;
    e->theta = new_theta;
  } else {
    // Stop the robot if blocked
    e->vx = 0;
    e->vy = 0;
    e->omega = 0;
  }
}

/**
 * @brief Smoothly rotates sim->head_rotation towards sim->target_head_angle by at most max_step per call.
 * Keeps angles normalized and handles wrap-around.
 */
void update_head_rotation(SimEntity* sim) {
  // Compute shortest angular difference
  double diff = sim->target_head_angle - sim->head_rotation;
  // Normalize diff to [-PI, PI]
  while (diff > PI) {
    diff -= TWO_PI;
  }
  while (diff < -PI) {
    diff += TWO_PI;
  }
  double max_step = 0.05;   // radians per frame (tune as needed)
  if (fabs(diff) > 1e-3) {  // Only update if not already close
    if (fabs(diff) < max_step) {
      sim->head_rotation = sim->target_head_angle;
    } else {
      sim->head_rotation += (diff > 0 ? max_step : -max_step);
      // Normalize to [0, 2PI]
      if (sim->head_rotation < 0) sim->head_rotation += TWO_PI;
      if (sim->head_rotation > TWO_PI) sim->head_rotation -= TWO_PI;
    }
  } else {
    sim->head_at_target = true;
  }  // If we reach the target angle, set flag
}

// --- Texture-based occupancy grid rendering ---
#define K_OCCUPIED_COLOR 0x000000FF  // Black, opaque
#define K_FREE_COLOR 0xFFFFFFFF      // White, opaque

static SDL_Texture* occupancy_grid_texture = NULL;
static int occupancy_grid_tex_w = 0;
static int occupancy_grid_tex_h = 0;
static int occupancy_grid_dirty = 1;  // Set to 1 when grid changes

static void update_occupancy_grid_texture(SDL_Renderer* renderer, bool** grid, int w, int h) {
  if (w <= 0 || h <= 0) {
    printf("Error: Invalid occupancy grid dimensions (width: %d, height: %d)\n", w, h);
    return;
  }

  if (!occupancy_grid_dirty && occupancy_grid_texture && occupancy_grid_tex_w == w && occupancy_grid_tex_h == h) return;

  if (occupancy_grid_texture) {
    SDL_DestroyTexture(occupancy_grid_texture);
    occupancy_grid_texture = NULL;
  }

  occupancy_grid_texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STATIC, w, h);
  if (!occupancy_grid_texture) {
    printf("Failed to create occupancy grid texture: %s\n", SDL_GetError());
    return;
  }

  occupancy_grid_tex_w = w;
  occupancy_grid_tex_h = h;
  Uint32* pixels = (Uint32*)malloc(w * h * sizeof(Uint32));
  if (!pixels) return;

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      pixels[(y * w) + x] = grid[x][y] ? K_OCCUPIED_COLOR : K_FREE_COLOR;
    }
  }

  SDL_UpdateTexture(occupancy_grid_texture, NULL, pixels, w * sizeof(Uint32));
  free(pixels);
  occupancy_grid_dirty = 0;
}

void draw_occupancy_grid(SDL_Renderer* renderer, bool** grid, int w, int h) {
  // Fill the entire window with black before drawing the map
  int win_w = 0;
  int win_h = 0;
  SDL_GetRendererOutputSize(renderer, &win_w, &win_h);
  SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);  // Black
  SDL_RenderClear(renderer);
  update_occupancy_grid_texture(renderer, grid, w, h);
  if (!occupancy_grid_texture) return;
  SDL_Rect dst_rect;
  dst_rect.x = (int)pan_x;
  dst_rect.y = (int)pan_y;
  dst_rect.w = (int)(w * zoom);
  dst_rect.h = (int)(h * zoom);
  SDL_RenderCopy(renderer, occupancy_grid_texture, NULL, &dst_rect);
}

static void mark_occupancy_grid_dirty() { occupancy_grid_dirty = 1; }

/**
 * @brief Simulates 2D laser scan from robot's position
 * @param renderer SDL renderer context for visualization
 * @param e Entity (robot) performing the scan
 * @param grid Occupancy grid
 * @param map_w Map width in pixels
 * @param map_h Map height in pixels
 * @param config Map configuration parameters
 * @param sim_entity SimEntity struct with per-entity laser params
 */
void laser_scan(SDL_Renderer* renderer, Entity* e, bool** grid, int map_w, int map_h, MapYamlConfig* config, SimEntity* sim_entity) {
  int robot_px;
  int robot_py;
  world_to_map_coords(e->x, e->y, &robot_px, &robot_py, config, map_h);
  int robot_sx;
  int robot_sy;
  map_to_screen(robot_px, robot_py, &robot_sx, &robot_sy);
  if (!sim_entity->laser_data) {
    return;
  }
  double laser_angle = sim_entity->laser_angle;
  if (laser_angle > TWO_PI) {
    laser_angle = TWO_PI;
  }
  if (laser_angle < 0) {
    laser_angle = 0;
  }
  int n_rays = sim_entity->laser_resolution > 1 ? sim_entity->laser_resolution : 2;
  double angle_min = -laser_angle / 2.0;
  double angle_increment = laser_angle / (n_rays - 1);
  for (int i = 0; i < n_rays; i++) {
    sim_entity->laser_data[i] = sim_entity->laser_range;
    double angle = angle_min + (i * angle_increment) + e->theta;
    // printf("angle = %f\n", angle);
    if (angle > TWO_PI) {
      angle -= TWO_PI;
    }
    if (angle < 0) {
      angle += TWO_PI;
    }
    double wx = e->x + (sim_entity->laser_range * cos(angle));
    double wy = e->y + (sim_entity->laser_range * sin(angle));
    int end_x;
    int end_y;
    world_to_map_coords(wx, wy, &end_x, &end_y, config, map_h);
    int x0 = robot_px;
    int y0 = robot_py;
    int x1 = end_x;
    int y1 = end_y;
    int dx = abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    int e2;
    int x = x0;
    int y = y0;
    int prev_x = x0;
    int prev_y = y0;
    int hit = 0;
    int steps = 0;
    while (true) {
      if (x < 0 || x >= map_w || y < 0 || y >= map_h) break;
      if (grid[x][y]) {
        double dist = sqrt(((prev_x - x0) * (prev_x - x0)) + ((prev_y - y0) * (prev_y - y0)));
        double range = dist * config->resolution;

        if (range < 0.0) {
          range = 0.0;
        }
        if (range >= sim_entity->laser_range) {
          range = sim_entity->laser_range;
        }
        sim_entity->laser_data[i] = range;
        int prev_sx;
        int prev_sy;
        map_to_screen(prev_x, prev_y, &prev_sx, &prev_sy);
        if (RENDER_SIM) {
          SDL_SetRenderDrawColor(renderer, 0, COLOR_ALPHA_OPAQUE, 0, COLOR_ALPHA_OPAQUE);
          SDL_RenderDrawLineF(renderer, robot_sx, robot_sy, prev_sx, prev_sy);
        }
        hit = 1;
        break;
      }
      prev_x = x;
      prev_y = y;
      if (x == x1 && y == y1) break;
      e2 = 2 * err;
      if (e2 >= dy) {
        err += dy;
        x += sx;
      }
      if (e2 <= dx) {
        err += dx;
        y += sy;
      }
      steps++;
      if (steps > (int)(sim_entity->laser_range / config->resolution)) break;
    }
    if (!hit) {
      int prev_sx;
      int prev_sy;
      map_to_screen(prev_x, prev_y, &prev_sx, &prev_sy);
      if (RENDER_SIM) {
        SDL_SetRenderDrawColor(renderer, 0, COLOR_ALPHA_OPAQUE, 0, COLOR_ALPHA_OPAQUE);
        SDL_RenderDrawLineF(renderer, robot_sx, robot_sy, prev_sx, prev_sy);
      }
    }
  }
}

static MapYamlConfig g_config;

EXPORT void set_map_params(const char* image, double resolution, double* origin, int negate, double occupied_thresh, double free_thresh, double speed, double angular_speed) {
  strncpy(g_config.image, image, sizeof(g_config.image) - 1);
  g_config.image[sizeof(g_config.image) - 1] = '\0';
  g_config.resolution = resolution;
  g_config.origin[0] = origin[0];
  g_config.origin[1] = origin[1];
  g_config.origin[2] = origin[2];
  g_config.negate = negate;
  g_config.occupied_thresh = occupied_thresh;
  g_config.free_thresh = free_thresh;
  g_config.speed = speed;
  g_config.angular_speed = angular_speed;
}

/**
 * @brief Debug function to print entity coordinates in both world and map frames
 * @param e Entity to print coordinates for
 * @param config Map configuration parameters
 * @param map_h Map height in pixels
 * @param name Name identifier for the entity
 */
void print_entity_pixel_coords(Entity* e, MapYamlConfig* config, int map_h, const char* name) {
  int px;
  int py;
  world_to_map_coords(e->x, e->y, &px, &py, config, map_h);
  printf("%s world: (%.2f, %.2f) -> map px: (%d, %d)\n", name, e->x, e->y, px, py);
}

/**
 * @brief Updates the entity's velocity and angular velocity based on keyboard input
 * @param e Entity to control
 * @param speed Linear speed (m/s) for up/down/left/right
 * @param angular_speed Angular speed (rad/s) for A/D
 */
void keyboard_control(Entity* e, double speed, double angular_speed) {
  const Uint8* state = SDL_GetKeyboardState(NULL);
  e->vx = 0;
  e->vy = 0;
  e->omega = 0;
  if (state[SDL_SCANCODE_I]) {
    e->vx = speed;
  }
  if (state[SDL_SCANCODE_K]) {
    e->vx = -speed;
  }
  if (state[SDL_SCANCODE_J]) {
    e->vy = speed;
  }
  if (state[SDL_SCANCODE_L]) {
    e->vy = -speed;
  }
  if (state[SDL_SCANCODE_D] && !(SDL_GetModState() & KMOD_SHIFT)) {
    e->omega = -angular_speed;
  }
  if (state[SDL_SCANCODE_A] && !(SDL_GetModState() & KMOD_SHIFT)) {
    e->omega = angular_speed;
  }
}

/**
 * @brief Updates and draws a SimEntity (handles keyboard, movement, and rendering)
 * @param sim SimEntity pointer
 * @param config Map configuration parameters
 * @param occupancy_grid Occupancy grid
 * @param map_w Map width in pixels
 * @param map_h Map height in pixels
 * @param renderer SDL renderer context
 */
void update_and_draw_entity(SimEntity* sim, MapYamlConfig* config, bool** occupancy_grid, int map_w, int map_h, SDL_Renderer* renderer) {
  if (sim->use_keyboard && sim->entity.vx == 0 && sim->entity.omega == 0) {
    // Update head_rotation for keyboard-controlled entity
    const Uint8* state = SDL_GetKeyboardState(NULL);
    if (state[SDL_SCANCODE_A] && (SDL_GetModState() & KMOD_SHIFT)) {
      sim->head_rotation -= 0.05;
      if (sim->head_rotation < 0) {
        sim->head_rotation += TWO_PI;
      }
    }
    if (state[SDL_SCANCODE_D] && (SDL_GetModState() & KMOD_SHIFT)) {
      sim->head_rotation += 0.05;
      if (sim->head_rotation > TWO_PI) {
        sim->head_rotation -= TWO_PI;
      }
    }
    keyboard_control(&sim->entity, config->speed, config->angular_speed);
    sim->last_vel_command = SDL_GetTicks();  // Update timestamp for keyboard control
  } else {
    // Check for velocity timeout
    Uint32 current_time = SDL_GetTicks();
    if (current_time - sim->last_vel_command > VELOCITY_TIMEOUT_MS) {
      // Zero out velocities if no recent command
      sim->entity.vx = 0;
      sim->entity.vy = 0;
      sim->entity.omega = 0;
    }
  }

  if (sim->use_differential) {
    move_entity_differential(&sim->entity, 1.0 / FPS, occupancy_grid, map_w, map_h, config);
  } else {
    move_entity_holonomic(&sim->entity, 1.0 / FPS, occupancy_grid, map_w, map_h, config);
  }

  // Update head rotation if not at target
  if (!sim->head_at_target) {
    update_head_rotation(sim);
  }
  if (RENDER_SIM) {
    draw_entity(renderer, &sim->entity, config, map_h, sim->head_rotation);
  }
}

/**
 * @brief Add a SimEntity from Python/YAML
 * @param x X position
 * @param y Y position
 * @param theta Orientation
 * @param radius Collision radius
 * @param color RGBA int array (length 4)
 * @param use_keyboard Keyboard control flag
 * @param use_differential Differential drive flag
 * @param use_laser Laser scan flag
 */
EXPORT void add_entity(double x, double y, double theta, double radius, const int* color, bool use_keyboard, bool use_differential, bool use_laser, const char* type, double laser_range,
                       int laser_resolution, double laser_angle) {
  if (use_keyboard) {
    if (keyboard_in_use) {
      printf("Warning: Only one entity can have use_keyboard=true. Entity '%s' will have keyboard disabled.\n", type ? type : "unknown");
      use_keyboard = false;
    } else {
      keyboard_in_use = true;
    }
  }
  if (g_entity_count >= MAX_ENTITIES) return;
  SimEntity* e = &g_entities[g_entity_count++];
  e->entity.x = x;
  e->entity.y = y;
  e->entity.theta = theta;
  e->entity.radius = radius;
  e->entity.color.r = color[0];
  e->entity.color.g = color[1];
  e->entity.color.b = color[2];
  e->entity.color.a = color[3];
  e->entity.vx = 0;
  e->entity.vy = 0;
  e->entity.omega = 0;
  e->initial_x = x;
  e->initial_y = y;
  e->initial_theta = theta;
  e->use_keyboard = use_keyboard;
  e->use_differential = use_differential;
  e->use_laser = use_laser;
  e->laser_range = laser_range;
  e->laser_resolution = laser_resolution;
  e->laser_angle = laser_angle;
  e->laser_data = NULL;
  e->last_vel_command = SDL_GetTicks();
  e->head_rotation = 0.0;      // Initialize rotation state
  e->target_head_angle = 0.0;  // Initialize target head angle
  e->head_at_target = true;    // Start with head at target angle
  if (use_laser && laser_resolution > 0) {
    e->laser_data = (float*)malloc(laser_resolution * sizeof(float));
    if (e->laser_data) {
      if (laser_range <= 0.0) {
        laser_range = LASER_DEFAULT_RANGE;
        printf("Warning: Invalid laser range provided, defaulting to %.1f meters\n", laser_range);
      }
      for (int i = 0; i < laser_resolution; i++) {
        e->laser_data[i] = laser_range;
      }
    } else {
      printf("Warning: Failed to allocate laser data array for entity '%s'\n", type ? type : "unknown");
      e->use_laser = false;
    }
  } else {
    e->laser_data = NULL;
  }
  e->last_vel_command = SDL_GetTicks();  // Initialize timestamp
}

EXPORT void run_simulation() {
  XInitThreads();  // Initialize X11 threading for SDL compatibility
  // Register SIGINT handler for graceful shutdown
  signal(SIGINT, handle_sigint);

  SDL_Window* window = NULL;
  SDL_Renderer* renderer = NULL;

  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    printf("SDL could not initialize! SDL_Error: %s\n", SDL_GetError());
    return;
  }
  int map_w;
  int map_h;
  MapYamlConfig* config = &g_config;
  bool** occupancy_grid = load_occupancy_grid_pgm(config->image, &map_w, &map_h);
  if (!occupancy_grid) return;
  SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");  // Enable linear filtering
  if (RENDER_SIM) {
    window = SDL_CreateWindow("Cohan Tutorial Sim", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, map_w, map_h, SDL_WINDOW_SHOWN);
  }
  renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

  bool quit = false;
  SDL_Event e;
  Uint32 start_tick;
  while (!quit) {
    if (sigint_received) {
      printf("\nSIGINT received, shutting down simulation...\n");
      quit = true;
      break;
    }
    start_tick = SDL_GetTicks();
    while (SDL_PollEvent(&e) != 0) {
      if (e.type == SDL_QUIT) {
        quit = true;
      }
      if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_r) {
        reset_all_entities();
      }
      // Reset zoom and pan with 'z' key
      if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_z) {
        zoom = 1.0;
        pan_x = 0.0;
        pan_y = 0.0;
      }
      // Mouse wheel for zoom (centered on mouse)
      if (e.type == SDL_MOUSEWHEEL) {
        int mouse_x;
        int mouse_y;
        SDL_GetMouseState(&mouse_x, &mouse_y);
        double prev_zoom = zoom;
        if (e.wheel.y > 0) zoom *= ZOOM_STEP;
        if (e.wheel.y < 0) zoom /= ZOOM_STEP;
        pan_x = mouse_x - ((mouse_x - pan_x) * (zoom / prev_zoom));
        pan_y = mouse_y - ((mouse_y - pan_y) * (zoom / prev_zoom));
      }
      // Mouse button for pan start
      if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT) {
        dragging = 1;
        drag_start_x = e.button.x;
        drag_start_y = e.button.y;
        pan_start_x = pan_x;
        pan_start_y = pan_y;
      }
      if (e.type == SDL_MOUSEBUTTONUP && e.button.button == SDL_BUTTON_LEFT) {
        dragging = 0;
      }
      if (e.type == SDL_MOUSEMOTION && dragging) {
        pan_x = pan_start_x + (e.motion.x - drag_start_x);
        pan_y = pan_start_y + (e.motion.y - drag_start_y);
      }
    }
    // Draw occupancy grid and laser scan before entities
    if (RENDER_SIM) {
      draw_occupancy_grid(renderer, occupancy_grid, map_w, map_h);
    }

    for (int i = 0; i < g_entity_count; ++i) {
      if (g_entities[i].use_laser) {
        laser_scan(renderer, &g_entities[i].entity, occupancy_grid, map_w, map_h, config, &g_entities[i]);
      }
    }
    // Update, move, and draw all entities
    for (int i = 0; i < g_entity_count; ++i) {
      update_and_draw_entity(&g_entities[i], config, occupancy_grid, map_w, map_h, renderer);
    }
    SDL_RenderPresent(renderer);
    // Cap FPS
    Uint32 elapsed = SDL_GetTicks() - start_tick;
    if (elapsed < MS_PER_SECOND / FPS) {
      SDL_Delay((MS_PER_SECOND / FPS) - elapsed);
    }
  }
  // Cleanup SDL resources
  // Destroy SDL resources first
  if (renderer) {
    SDL_DestroyRenderer(renderer);
    renderer = NULL;
  }
  if (window) {
    SDL_DestroyWindow(window);
    window = NULL;
  }
  SDL_Quit();

  // Free laser data arrays for all entities
  for (int i = 0; i < g_entity_count; i++) {
    if (g_entities[i].laser_data) {
      free(g_entities[i].laser_data);
      g_entities[i].laser_data = NULL;
    }
  }

  g_entity_count = 0;
  keyboard_in_use = false;

  // Free global occupancy grid
  if (occupancy_grid) {
    for (int x = 0; x < map_w; x++) {
      if (occupancy_grid[x]) {
        free(occupancy_grid[x]);
        occupancy_grid[x] = NULL;
      }
    }
    free(occupancy_grid);
    occupancy_grid = NULL;
  }

  printf("Simulation cleanup complete. Exiting run_simulation().\n");
}

// Global variables for step-based simulation
static bool** g_occupancy_grid = NULL;
static int g_map_w = 0;
static int g_map_h = 0;
static bool g_step_initialized = false;

// Helper to initialize occupancy grid for step_simulation
static void ensure_step_sim_initialized() {
  if (!g_step_initialized) {
    MapYamlConfig* config = &g_config;
    if (!g_occupancy_grid) {
      g_occupancy_grid = load_occupancy_grid_pgm(config->image, &g_map_w, &g_map_h);
    }
    g_step_initialized = true;
  }
}

EXPORT void render_simulation(SDL_Renderer* step_renderer) {
  MapYamlConfig* config = &g_config;

  // Render the map
  draw_occupancy_grid(step_renderer, g_occupancy_grid, g_map_w, g_map_h);

  // Render laser scans
  for (int i = 0; i < g_entity_count; ++i) {
    if (g_entities[i].use_laser) {
      laser_scan(step_renderer, &g_entities[i].entity, g_occupancy_grid, g_map_w, g_map_h, config, &g_entities[i]);
    }
  }

  // Render entities
  for (int i = 0; i < g_entity_count; ++i) {
    SimEntity* sim = &g_entities[i];
    draw_entity(step_renderer, &sim->entity, config, g_map_h, sim->head_rotation);
  }
}

EXPORT void step_simulation() {
  ensure_step_sim_initialized();

  MapYamlConfig* config = &g_config;
  double dt = 1.0 / FPS;

  // Update laser scans first
  for (int i = 0; i < g_entity_count; ++i) {
    if (g_entities[i].use_laser) {
      laser_scan(NULL, &g_entities[i].entity, g_occupancy_grid, g_map_w, g_map_h, config, &g_entities[i]);
    }
  }

  // Update, move all entities
  for (int i = 0; i < g_entity_count; ++i) {
    SimEntity* sim = &g_entities[i];
    // Velocity timeout
    Uint32 current_time = SDL_GetTicks();
    if (current_time - sim->last_vel_command > VELOCITY_TIMEOUT_MS) {
      sim->entity.vx = 0;
      sim->entity.vy = 0;
      sim->entity.omega = 0;
    }
    if (sim->use_differential) {
      move_entity_differential(&sim->entity, dt, g_occupancy_grid, g_map_w, g_map_h, config);
    } else {
      move_entity_holonomic(&sim->entity, dt, g_occupancy_grid, g_map_w, g_map_h, config);
    }
    // Update head rotation if not at target
    if (!sim->head_at_target) {
      update_head_rotation(sim);
    }
  }
}

// Global variables for robot state
static double robot_states[MAX_ENTITIES][ROBOT_STATE_SIZE];  // [x, y, theta, vx, vth] for each entity

/**
 * @brief Get the current state of a specific robot (entity) in the simulation
 * @param agent_idx Index of the agent to get state for
 * @return Pointer to array containing [x, y, theta, vx, vth]
 */
EXPORT double* get_robot_state(int agent_idx) {
  if (agent_idx >= 0 && agent_idx < g_entity_count) {
    robot_states[agent_idx][0] = g_entities[agent_idx].entity.x;
    robot_states[agent_idx][1] = g_entities[agent_idx].entity.y;
    robot_states[agent_idx][2] = g_entities[agent_idx].entity.theta;
    robot_states[agent_idx][3] = g_entities[agent_idx].entity.vx;
    robot_states[agent_idx][4] = g_entities[agent_idx].entity.vy;
    robot_states[agent_idx][5] = g_entities[agent_idx].entity.omega;
    robot_states[agent_idx][6] = g_entities[agent_idx].head_rotation;
    return robot_states[agent_idx];
  }
  return NULL;
}

/**
 * @brief Set velocities for a specific robot (entity) in the simulation
 * @param agent_idx Index of the agent to set velocities for
 * @param linear_vel Linear velocity in m/s (in robot's local frame)
 * @param angular_vel Angular velocity in rad/s
 */
EXPORT void set_robot_velocity(int agent_idx, double vel_x, double vel_y, double angular_vel) {
  if (agent_idx >= 0 && agent_idx < g_entity_count) {
    SimEntity* e = &g_entities[agent_idx];
    e->last_vel_command = SDL_GetTicks();  // Update timestamp when velocity command received
    e->entity.vx = vel_x;
    e->entity.vy = vel_y;
    e->entity.omega = angular_vel;
  }
}

/**
 * @brief Set the head rotation for a specific agent
 * @param agent_idx Index of the agent
 * @param angle Rotation angle in radians
 */
EXPORT void set_head_rotation(int agent_idx, double angle) {
  if (agent_idx >= 0 && agent_idx < g_entity_count) {
    SimEntity* e = &g_entities[agent_idx];
    // Normalize angle to [0, 2π]
    if (angle < 0) {
      angle += TWO_PI;
    }
    if (angle > TWO_PI) {
      angle -= TWO_PI;
    }
    e->target_head_angle = angle;
    e->head_at_target = false;  // Reset flag since we are setting a new target
  }
}

/**
 * @brief Get laser scan data for a specific robot
 * @param agent_idx Index of the agent to get laser data for
 * @return Pointer to array containing range measurements
 */
EXPORT float* get_laser_data(int agent_idx) {
  if (agent_idx < 0 || agent_idx >= g_entity_count || !g_entities[agent_idx].use_laser) {
    return NULL;
  }
  SimEntity* robot = &g_entities[agent_idx];
  if (!robot->laser_data || robot->laser_resolution <= 0) {
    printf("Warning: Invalid laser data configuration for agent %d\n", agent_idx);
    return NULL;
  }
  for (int i = 0; i < robot->laser_resolution; i++) {
    float val = robot->laser_data[i];
    if (val < 0.0F) {
      val = 0.0F;
    }
    if (val > (float)robot->laser_range) {
      val = (float)robot->laser_range;
    }
    (robot->laser_data)[i] = val;
  }
  return robot->laser_data;
}

static void reset_all_entities() {
  for (int i = 0; i < g_entity_count; ++i) {
    g_entities[i].entity.x = g_entities[i].initial_x;
    g_entities[i].entity.y = g_entities[i].initial_y;
    g_entities[i].entity.theta = g_entities[i].initial_theta;
    g_entities[i].entity.vx = 0;
    g_entities[i].entity.vy = 0;
    g_entities[i].entity.omega = 0;
    g_entities[i].last_vel_command = SDL_GetTicks();
  }
  // Reset zoom and pan
  zoom = 1.0;
  pan_x = 0.0;
  pan_y = 0.0;
}

// SIGINT handler implementation
static void handle_sigint(int sig) {
  (void)sig;
  sigint_received = 1;
}

static SDL_Window* render_window = NULL;
static SDL_Renderer* render_renderer = NULL;
static SDL_mutex* render_mutex = NULL;

static void initialize_render_resources() {
  if (!render_window && !render_renderer) {
    render_window = SDL_CreateWindow("Simulation Render", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, g_map_w, g_map_h, SDL_WINDOW_SHOWN);
    if (!render_window) {
      printf("Failed to create SDL window: %s\n", SDL_GetError());
      return;
    }
    render_renderer = SDL_CreateRenderer(render_window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!render_renderer) {
      printf("Failed to create SDL renderer: %s\n", SDL_GetError());
      SDL_DestroyWindow(render_window);
      render_window = NULL;
    }
  }
}

static void cleanup_render_resources() {
  if (render_renderer) {
    SDL_DestroyRenderer(render_renderer);
    render_renderer = NULL;
  }
  if (render_window) {
    SDL_DestroyWindow(render_window);
    render_window = NULL;
  }
}

static pthread_t render_thread;
static bool render_thread_running = false;

static void* render_thread_function(void* arg) {
  XInitThreads();  // Initialize X11 threading for SDL compatibility

  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    printf("SDL could not initialize in render thread! SDL_Error: %s\n", SDL_GetError());
    return NULL;
  }

  MapYamlConfig* config = &g_config;
  bool** occupancy_grid = load_occupancy_grid_pgm(config->image, &g_map_w, &g_map_h);
  if (!occupancy_grid) {
    return NULL;
  }

  SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");  // Enable linear filtering

  initialize_render_resources();

  if (!render_window || !render_renderer) {
    printf("Failed to initialize SDL resources in render thread.\n");
    SDL_Quit();
    return NULL;
  }

  // SDL_ShowWindow(render_window);  // Ensure the window is visible

  SDL_Event e;
  Uint32 start_tick;
  while (render_thread_running) {
    // Handle events in the rendering thread
    start_tick = SDL_GetTicks();
    while (SDL_PollEvent(&e)) {
      if (e.type == SDL_QUIT) {
        render_thread_running = false;
      }
      if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_r) {
        reset_all_entities();
      }
      // Reset zoom and pan with 'z' key
      if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_z) {
        zoom = 1.0;
        pan_x = 0.0;
        pan_y = 0.0;
      }
      // Mouse wheel for zoom (centered on mouse)
      if (e.type == SDL_MOUSEWHEEL) {
        int mouse_x;
        int mouse_y;
        SDL_GetMouseState(&mouse_x, &mouse_y);
        double prev_zoom = zoom;
        if (e.wheel.y > 0) zoom *= ZOOM_STEP;
        if (e.wheel.y < 0) zoom /= ZOOM_STEP;
        pan_x = mouse_x - ((mouse_x - pan_x) * (zoom / prev_zoom));
        pan_y = mouse_y - ((mouse_y - pan_y) * (zoom / prev_zoom));
      }
      // Mouse button for pan start
      if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT) {
        dragging = 1;
        drag_start_x = e.button.x;
        drag_start_y = e.button.y;
        pan_start_x = pan_x;
        pan_start_y = pan_y;
      }
      if (e.type == SDL_MOUSEBUTTONUP && e.button.button == SDL_BUTTON_LEFT) {
        dragging = 0;
      }
      if (e.type == SDL_MOUSEMOTION && dragging) {
        pan_x = pan_start_x + (e.motion.x - drag_start_x);
        pan_y = pan_start_y + (e.motion.y - drag_start_y);
      }
    }
    // SDL_LockMutex(render_mutex);
    if (render_renderer) {
      render_simulation(render_renderer);
      SDL_RenderPresent(render_renderer);
    }
    // SDL_UnlockMutex(render_mutex);
    // Cap FPS
    Uint32 elapsed = SDL_GetTicks() - start_tick;
    if (elapsed < MS_PER_SECOND / FPS) {
      SDL_Delay((MS_PER_SECOND / FPS) - elapsed);
    }
  }

  cleanup_render_resources();
  SDL_Quit();
  return NULL;
}

EXPORT void start_render_thread() {
  if (!render_thread_running) {
    render_thread_running = true;
    pthread_create(&render_thread, NULL, render_thread_function, NULL);
  }
}

EXPORT void stop_render_thread() {
  if (render_thread_running) {
    render_thread_running = false;
    pthread_join(render_thread, NULL);
  }
}
