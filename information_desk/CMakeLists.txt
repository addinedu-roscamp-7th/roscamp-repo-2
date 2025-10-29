cmake_minimum_required(VERSION 3.19)
project(information_desk LANGUAGES CXX)

find_package(Qt6 6.4 REQUIRED COMPONENTS Core Widgets LinguistTools Network)

qt_standard_project_setup()

qt_add_executable(information_desk
    WIN32 MACOSX_BUNDLE
    main.cpp
    infodeskmain.cpp
    infodeskmain.h
    infodeskmain.ui
    booksearchdialog.h  booksearchdialog.ui
    booksearchdialog.cpp

    buttonDelegate.h
    buttonDelegate.cpp
    bookinfolocation.h bookinfolocation.cpp bookinfolocation.ui
    bookloan.h bookloan.cpp bookloan.ui
    dobbyloan.h dobbyloan.cpp dobbyloan.ui

)

qt_add_translations(
    information_desk
    TS_FILES information_desk_ko_KR.ts
)

target_link_libraries(information_desk
    PRIVATE
        Qt::Core
        Qt::Widgets
         Qt6::Network
)

include(GNUInstallDirs)

install(TARGETS information_desk
    BUNDLE  DESTINATION .
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

#qt_generate_deploy_app_script(
#    TARGET information_desk
#    OUTPUT_SCRIPT deploy_script
#    NO_UNSUPPORTED_PLATFORM_ERROR
#)
#install(SCRIPT ${deploy_script})
