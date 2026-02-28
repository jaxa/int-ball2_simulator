/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef MAIN_CAMERA_VIEW_CONTROLLER_H
#define MAIN_CAMERA_VIEW_CONTROLLER_H
#include <OgreVector.h>
#include <OgreQuaternion.h>
#include <rviz_common/frame_position_tracking_view_controller.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/vector_property.hpp>

namespace intball
{

/**
 * @brief newMainCameraViewController
 * @return
 */
rviz_common::ViewController *newMainCameraViewController();

/** @brief A first-person camera, controlled by yaw, pitch, and position. */
class MainCameraViewController : public rviz_common::FramePositionTrackingViewController
{
Q_OBJECT
public:
  MainCameraViewController();
  virtual ~MainCameraViewController();

  void onInitialize() override;

  void yaw( float angle );
  void pitch( float angle );
  void move( float x, float y, float z );

  void handleMouseEvent(rviz_common::ViewportMouseEvent& evt) override;

  void lookAt( const Ogre::Vector3& point ) override;

  void reset() override;

  void mimic( rviz_common::ViewController* source_view ) override;

  void update(float dt, float ros_dt) override;

protected:
  void onTargetFrameChanged(const Ogre::Vector3& old_reference_position, const Ogre::Quaternion& old_reference_orientation) override;

  void setPropertiesFromCamera( Ogre::Camera* source_camera );

  void updateCamera();

  Ogre::Quaternion getOrientation();

  rviz_common::properties::FloatProperty* yaw_property_;
  rviz_common::properties::FloatProperty* pitch_property_;
  rviz_common::properties::FloatProperty* roll_property_;
  rviz_common::properties::VectorProperty* position_property_;
};

} // end namespace intball

#endif // MAIN_CAMERA_VIEW_CONTROLLER_H
