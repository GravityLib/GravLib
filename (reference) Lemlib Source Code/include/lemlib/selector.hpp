#ifndef SELECTOR_HPP
#define SELECTOR_HPP

#include <iostream>
#include <stdexcept>
#include "api.h"
#include "liblvgl/core/lv_event.h"
#include "liblvgl/core/lv_obj.h"
#include "liblvgl/lvgl.h"
#include "liblvgl/core/lv_disp.h"
#include "liblvgl/extra/widgets/tabview/lv_tabview.h"
#include "liblvgl/misc/lv_anim.h"
#include "liblvgl/misc/lv_area.h"
#include "liblvgl/misc/lv_types.h"
#include "liblvgl/widgets/lv_btnmatrix.h"

namespace lemlib::selector {

// Enumeration to define the autonomous state
enum AutonState {
    NONE = 0,
    RED = 1,
    BLUE = -1
};

// Global variables for selector logic
extern AutonState autonState;
extern int auton;
extern int autonCount;

extern int autonStarted;

extern uint16_t currentRedButton;
extern uint16_t currentBlueButton;

extern pros::rtos::Task* tabWatcher_task;

constexpr int MAX_AUTONS = 100;
extern const char* redBtnmMap[MAX_AUTONS];
extern const char* blueBtnmMap[MAX_AUTONS];

extern lv_obj_t* tabview;
extern lv_obj_t* redBtnm;
extern lv_obj_t* blueBtnm;

// Function declarations
int textUpdate();
void log_error(const std::string& func_name, const std::string& msg);
uint16_t get_button_count(lv_obj_t* btnm);
void deselect_all_buttons(lv_obj_t* btnm);
void redBtnmAction(lv_event_t* e);
void blueBtnmAction(lv_event_t* e);
void tabWatcher(void* param);
void init(int default_auton, const char** redAutons, const char** blueAutons, const char** skillsAutons,
            const char** redDesc, const char** blueDesc, const char** skillsDesc);
void destroy();

} // namespace lemlib::selector

#endif // SELECTOR_HPP