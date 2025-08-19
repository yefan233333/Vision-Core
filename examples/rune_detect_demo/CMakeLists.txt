find_package(OpenCV REQUIRED)

VisCore_add_exe(rune_detect_demo
DEPENDS core math contour_proc feature rune_target camera rune_fan rune_center
EXTERNAL ${OpenCV_LIBS}
)