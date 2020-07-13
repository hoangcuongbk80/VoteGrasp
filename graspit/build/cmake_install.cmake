# Install script for directory: /home/hoang/OSS/VoteGrasp/graspit

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/usr/local/bin/graspit_simulator" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/bin/graspit_simulator")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/usr/local/bin/graspit_simulator"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/bin/graspit_simulator")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/bin" TYPE EXECUTABLE FILES "/home/hoang/OSS/VoteGrasp/graspit/build/graspit_simulator")
  if(EXISTS "$ENV{DESTDIR}/usr/local/bin/graspit_simulator" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/bin/graspit_simulator")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/usr/local/bin/graspit_simulator"
         OLD_RPATH "/home/hoang/OSS/VoteGrasp/graspit/build:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/local/bin/graspit_simulator")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/usr/local/lib/libgraspit.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/lib/libgraspit.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/usr/local/lib/libgraspit.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/libgraspit.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/lib" TYPE SHARED_LIBRARY FILES "/home/hoang/OSS/VoteGrasp/graspit/build/libgraspit.so")
  if(EXISTS "$ENV{DESTDIR}/usr/local/lib/libgraspit.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/lib/libgraspit.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/local/lib/libgraspit.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/graspit/barrett.h;/usr/local/include/graspit/pr2Gripper.h;/usr/local/include/graspit/m7.h;/usr/local/include/graspit/m7tool.h;/usr/local/include/graspit/robonaut.h;/usr/local/include/graspit/robotiq.h;/usr/local/include/graspit/humanHand.h;/usr/local/include/graspit/shadow.h;/usr/local/include/graspit/mcGrip.h;/usr/local/include/graspit/puma560.h;/usr/local/include/graspit/robot.h;/usr/local/include/graspit/graspitParser.h;/usr/local/include/graspit/body.h;/usr/local/include/graspit/bBox.h;/usr/local/include/graspit/bbox_inl.h;/usr/local/include/graspit/contact.h;/usr/local/include/graspit/softContact.h;/usr/local/include/graspit/pointContact.h;/usr/local/include/graspit/virtualContact.h;/usr/local/include/graspit/virtualContactOnObject.h;/usr/local/include/graspit/contactSetting.h;/usr/local/include/graspit/debug.h;/usr/local/include/graspit/dof.h;/usr/local/include/graspit/dynamics.h;/usr/local/include/graspit/dynJoint.h;/usr/local/include/graspit/dynamicsEngine.h;/usr/local/include/graspit/eigenGrasp.h;/usr/local/include/graspit/gloveInterface.h;/usr/local/include/graspit/grasp.h;/usr/local/include/graspit/graspRecord.h;/usr/local/include/graspit/gws.h;/usr/local/include/graspit/gwsprojection.h;/usr/local/include/graspit/ivmgr.h;/usr/local/include/graspit/jacobian.h;/usr/local/include/graspit/joint.h;/usr/local/include/graspit/kinematicChain.h;/usr/local/include/graspit/lmiOptimizer.h;/usr/local/include/graspit/material.h;/usr/local/include/graspit/matvec3D.h;/usr/local/include/graspit/matvecIO.h;/usr/local/include/graspit/maxdet.h;/usr/local/include/graspit/mytools.h;/usr/local/include/graspit/profiling.h;/usr/local/include/graspit/qhull_mutex.h;/usr/local/include/graspit/quality.h;/usr/local/include/graspit/plugin.h;/usr/local/include/graspit/SoArrow.h;/usr/local/include/graspit/SoComplexShape.h;/usr/local/include/graspit/SoTorquePointer.h;/usr/local/include/graspit/scanSimulator.h;/usr/local/include/graspit/timer_calls.h;/usr/local/include/graspit/triangle_inl.h;/usr/local/include/graspit/triangle.h;/usr/local/include/graspit/worldElement.h;/usr/local/include/graspit/worldElementFactory.h;/usr/local/include/graspit/world.h;/usr/local/include/graspit/graspitCore.h;/usr/local/include/graspit/graspitServer.h;/usr/local/include/graspit/graspitApp.h;/usr/local/include/graspit/arch.h;/usr/local/include/graspit/FitParabola.h;/usr/local/include/graspit/bodySensor.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/include/graspit" TYPE FILE FILES
    "/home/hoang/OSS/VoteGrasp/graspit/include/robots/barrett.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/robots/pr2Gripper.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/robots/m7.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/robots/m7tool.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/robots/robonaut.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/robots/robotiq.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/robots/humanHand.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/robots/shadow.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/robots/mcGrip.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/robots/puma560.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/robot.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/graspitParser.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/body.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/bBox.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/bbox_inl.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/contact/contact.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/contact/softContact.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/contact/pointContact.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/contact/virtualContact.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/contact/virtualContactOnObject.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/contactSetting.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/debug.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/dof.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/dynamics/dynamics.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/dynamics/dynJoint.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/dynamics/dynamicsEngine.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/eigenGrasp.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/gloveInterface.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/grasp.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/graspRecord.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/gws.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/gwsprojection.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/ivmgr.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/jacobian.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/joint.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/kinematicChain.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/lmiOptimizer.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/material.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/matvec3D.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/matvecIO.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/maxdet.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/mytools.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/profiling.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/qhull_mutex.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/quality.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/plugin.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/SoArrow.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/SoComplexShape.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/SoTorquePointer.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/scanSimulator.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/timer_calls.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/triangle_inl.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/triangle.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/worldElement.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/worldElementFactory.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/world.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/graspitCore.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/graspitServer.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/graspitApp.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/arch.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/FitParabola.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/bodySensor.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/graspit/cmdline/cmdline.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/include/graspit/cmdline" TYPE FILE FILES "/home/hoang/OSS/VoteGrasp/graspit/cmdline/cmdline.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/graspit/dynamics/graspitDynamics.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/include/graspit/dynamics" TYPE FILE FILES "/home/hoang/OSS/VoteGrasp/graspit/include/dynamics/graspitDynamics.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/graspit/Planner/grasp_visualization.h;/usr/local/include/graspit/Planner/grasp_tester.h;/usr/local/include/graspit/Planner/grasp_preshape.h;/usr/local/include/graspit/Planner/grasp_presenter.h;/usr/local/include/graspit/Planner/grasp_planner.h;/usr/local/include/graspit/Planner/grasp_manager.h;/usr/local/include/graspit/Planner/grasp_grasps.h;/usr/local/include/graspit/Planner/grasp_directions.h;/usr/local/include/graspit/Planner/grasp_coordinates.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/include/graspit/Planner" TYPE FILE FILES
    "/home/hoang/OSS/VoteGrasp/graspit/include/Planner/grasp_visualization.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/Planner/grasp_tester.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/Planner/grasp_preshape.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/Planner/grasp_presenter.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/Planner/grasp_planner.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/Planner/grasp_manager.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/Planner/grasp_grasps.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/Planner/grasp_directions.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/Planner/grasp_coordinates.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/graspit/EGPlanner/search.h;/usr/local/include/graspit/EGPlanner/simAnn.h;/usr/local/include/graspit/EGPlanner/searchState.h;/usr/local/include/graspit/EGPlanner/searchStateImpl.h;/usr/local/include/graspit/EGPlanner/onLinePlanner.h;/usr/local/include/graspit/EGPlanner/egPlanner.h;/usr/local/include/graspit/EGPlanner/simAnnPlanner.h;/usr/local/include/graspit/EGPlanner/guidedPlanner.h;/usr/local/include/graspit/EGPlanner/loopPlanner.h;/usr/local/include/graspit/EGPlanner/timeTest.h;/usr/local/include/graspit/EGPlanner/graspTesterThread.h;/usr/local/include/graspit/EGPlanner/onLineGraspInterface.h;/usr/local/include/graspit/EGPlanner/listPlanner.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/include/graspit/EGPlanner" TYPE FILE FILES
    "/home/hoang/OSS/VoteGrasp/graspit/include/EGPlanner/search.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/EGPlanner/simAnn.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/EGPlanner/searchState.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/EGPlanner/searchStateImpl.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/EGPlanner/onLinePlanner.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/EGPlanner/egPlanner.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/EGPlanner/simAnnPlanner.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/EGPlanner/guidedPlanner.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/EGPlanner/loopPlanner.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/EGPlanner/timeTest.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/EGPlanner/graspTesterThread.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/EGPlanner/onLineGraspInterface.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/EGPlanner/listPlanner.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/graspit/EGPlanner/energy/searchEnergy.h;/usr/local/include/graspit/EGPlanner/energy/autograspQualityEnergy.h;/usr/local/include/graspit/EGPlanner/energy/compliantEnergy.h;/usr/local/include/graspit/EGPlanner/energy/contactEnergy.h;/usr/local/include/graspit/EGPlanner/energy/dynamicAutograspEnergy.h;/usr/local/include/graspit/EGPlanner/energy/guidedAutoGraspEnergy.h;/usr/local/include/graspit/EGPlanner/energy/guidedPotentialQualityEnergy.h;/usr/local/include/graspit/EGPlanner/energy/potentialQualityEnergy.h;/usr/local/include/graspit/EGPlanner/energy/strictAutoGraspEnergy.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/include/graspit/EGPlanner/energy" TYPE FILE FILES
    "/home/hoang/OSS/VoteGrasp/graspit/include/EGPlanner/energy/searchEnergy.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/EGPlanner/energy/autograspQualityEnergy.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/EGPlanner/energy/compliantEnergy.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/EGPlanner/energy/contactEnergy.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/EGPlanner/energy/dynamicAutograspEnergy.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/EGPlanner/energy/guidedAutoGraspEnergy.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/EGPlanner/energy/guidedPotentialQualityEnergy.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/EGPlanner/energy/potentialQualityEnergy.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/EGPlanner/energy/strictAutoGraspEnergy.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/graspit/Collision/collisionInterface.h;/usr/local/include/graspit/Collision/collisionStructures.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/include/graspit/Collision" TYPE FILE FILES
    "/home/hoang/OSS/VoteGrasp/graspit/include/Collision/collisionInterface.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/Collision/collisionStructures.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/graspit/math/matrix.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/include/graspit/math" TYPE FILE FILES "/home/hoang/OSS/VoteGrasp/graspit/include/math/matrix.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/graspit/ui/ui_mainWindow.h;/usr/local/include/graspit/ui/ui_about.h;/usr/local/include/graspit/ui/ui_archBuilderDlg.h;/usr/local/include/graspit/ui/ui_barrettHandDlg.h;/usr/local/include/graspit/ui/ui_bodyPropDlg.h;/usr/local/include/graspit/ui/ui_contactExaminerDlg.h;/usr/local/include/graspit/ui/ui_eigenGraspDlg.h;/usr/local/include/graspit/ui/ui_gfoDlg.h;/usr/local/include/graspit/ui/ui_gloveCalibrationDlg.h;/usr/local/include/graspit/ui/ui_graspCaptureDlg.h;/usr/local/include/graspit/ui/ui_gwsProjDlgBase.h;/usr/local/include/graspit/ui/ui_qmDlg.h;/usr/local/include/graspit/ui/ui_qualityIndicator.h;/usr/local/include/graspit/ui/ui_settingsDlg.h;/usr/local/include/graspit/ui/ui_plannerdlg.h;/usr/local/include/graspit/ui/ui_egPlannerDlg.h;/usr/local/include/graspit/ui/ui_compliantPlannerDlg.h;/usr/local/include/graspit/ui/ui_dbaseDlg.h;/usr/local/include/graspit/ui/ui_dbasePlannerDlg.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/include/graspit/ui" TYPE FILE FILES
    "/home/hoang/OSS/VoteGrasp/graspit/build/include/graspit/ui/ui_mainWindow.h"
    "/home/hoang/OSS/VoteGrasp/graspit/build/include/graspit/ui/ui_about.h"
    "/home/hoang/OSS/VoteGrasp/graspit/build/include/graspit/ui/ui_archBuilderDlg.h"
    "/home/hoang/OSS/VoteGrasp/graspit/build/include/graspit/ui/ui_barrettHandDlg.h"
    "/home/hoang/OSS/VoteGrasp/graspit/build/include/graspit/ui/ui_bodyPropDlg.h"
    "/home/hoang/OSS/VoteGrasp/graspit/build/include/graspit/ui/ui_contactExaminerDlg.h"
    "/home/hoang/OSS/VoteGrasp/graspit/build/include/graspit/ui/ui_eigenGraspDlg.h"
    "/home/hoang/OSS/VoteGrasp/graspit/build/include/graspit/ui/ui_gfoDlg.h"
    "/home/hoang/OSS/VoteGrasp/graspit/build/include/graspit/ui/ui_gloveCalibrationDlg.h"
    "/home/hoang/OSS/VoteGrasp/graspit/build/include/graspit/ui/ui_graspCaptureDlg.h"
    "/home/hoang/OSS/VoteGrasp/graspit/build/include/graspit/ui/ui_gwsProjDlgBase.h"
    "/home/hoang/OSS/VoteGrasp/graspit/build/include/graspit/ui/ui_qmDlg.h"
    "/home/hoang/OSS/VoteGrasp/graspit/build/include/graspit/ui/ui_qualityIndicator.h"
    "/home/hoang/OSS/VoteGrasp/graspit/build/include/graspit/ui/ui_settingsDlg.h"
    "/home/hoang/OSS/VoteGrasp/graspit/build/include/graspit/ui/ui_plannerdlg.h"
    "/home/hoang/OSS/VoteGrasp/graspit/build/include/graspit/ui/ui_egPlannerDlg.h"
    "/home/hoang/OSS/VoteGrasp/graspit/build/include/graspit/ui/ui_compliantPlannerDlg.h"
    "/home/hoang/OSS/VoteGrasp/graspit/build/include/graspit/ui/ui_dbaseDlg.h"
    "/home/hoang/OSS/VoteGrasp/graspit/build/include/graspit/ui/ui_dbasePlannerDlg.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/graspit/DBase/graspit_db_model.h;/usr/local/include/graspit/DBase/graspit_db_grasp.h;/usr/local/include/graspit/DBase/graspit_db_planner.h;/usr/local/include/graspit/DBase/taskDispatcher.h;/usr/local/include/graspit/DBase/preGraspCheckTask.h;/usr/local/include/graspit/DBase/graspClusteringTask.h;/usr/local/include/graspit/DBase/graspTransferCheckTask.h;/usr/local/include/graspit/DBase/tableCheckTask.h;/usr/local/include/graspit/DBase/compliantGraspCopyTask.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/include/graspit/DBase" TYPE FILE FILES
    "/home/hoang/OSS/VoteGrasp/graspit/include/DBase/graspit_db_model.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/DBase/graspit_db_grasp.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/DBase/graspit_db_planner.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/DBase/taskDispatcher.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/DBase/preGraspCheckTask.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/DBase/graspClusteringTask.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/DBase/graspTransferCheckTask.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/DBase/tableCheckTask.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/DBase/compliantGraspCopyTask.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/graspit/DBase/DBPlanner/grasp.h;/usr/local/include/graspit/DBase/DBPlanner/model.h;/usr/local/include/graspit/DBase/DBPlanner/db_manager.h;/usr/local/include/graspit/DBase/DBPlanner/database.h;/usr/local/include/graspit/DBase/DBPlanner/sql_database_manager.h;/usr/local/include/graspit/DBase/DBPlanner/task.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/include/graspit/DBase/DBPlanner" TYPE FILE FILES
    "/home/hoang/OSS/VoteGrasp/graspit/include/DBase/DBPlanner/grasp.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/DBase/DBPlanner/model.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/DBase/DBPlanner/db_manager.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/DBase/DBPlanner/database.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/DBase/DBPlanner/sql_database_manager.h"
    "/home/hoang/OSS/VoteGrasp/graspit/include/DBase/DBPlanner/task.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/graspit/ply/ply.h;/usr/local/include/graspit/ply/mesh_loader.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/include/graspit/ply" TYPE FILE FILES
    "/home/hoang/OSS/VoteGrasp/graspit/ply/ply.h"
    "/home/hoang/OSS/VoteGrasp/graspit/ply/mesh_loader.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/usr/local/lib/libgraspit.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/lib/libgraspit.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/usr/local/lib/libgraspit.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/libgraspit.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/lib" TYPE SHARED_LIBRARY PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_READ WORLD_READ FILES "/home/hoang/OSS/VoteGrasp/graspit/build/libgraspit.so")
  if(EXISTS "$ENV{DESTDIR}/usr/local/lib/libgraspit.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/lib/libgraspit.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/local/lib/libgraspit.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/usr/local/lib/graspit-targets.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}/usr/local/lib/graspit-targets.cmake"
         "/home/hoang/OSS/VoteGrasp/graspit/build/CMakeFiles/Export/_usr/local/lib/graspit-targets.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}/usr/local/lib/graspit-targets-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}/usr/local/lib/graspit-targets.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/graspit-targets.cmake")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/lib" TYPE FILE FILES "/home/hoang/OSS/VoteGrasp/graspit/build/CMakeFiles/Export/_usr/local/lib/graspit-targets.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/usr/local/lib/graspit-targets-release.cmake")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "/usr/local/lib" TYPE FILE FILES "/home/hoang/OSS/VoteGrasp/graspit/build/CMakeFiles/Export/_usr/local/lib/graspit-targets-release.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/graspit/FindBULLET.cmake;/usr/local/lib/graspit/FindSoQt4.cmake;/usr/local/lib/graspit/FindQhull.cmake")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/lib/graspit" TYPE FILE FILES
    "/home/hoang/OSS/VoteGrasp/graspit/CMakeMacros/FindBULLET.cmake"
    "/home/hoang/OSS/VoteGrasp/graspit/CMakeMacros/FindSoQt4.cmake"
    "/home/hoang/OSS/VoteGrasp/graspit/CMakeMacros/FindQhull.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/hoang/OSS/VoteGrasp/graspit/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
