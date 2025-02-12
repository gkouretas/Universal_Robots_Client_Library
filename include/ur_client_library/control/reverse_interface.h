// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
// Created on behalf of Universal Robots A/S
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Tristan Schnell schnell@fzi.de
 * \date    2019-04-11
 *
 */
//----------------------------------------------------------------------

#ifndef UR_CLIENT_LIBRARY_REVERSE_INTERFACE_H_INCLUDED
#define UR_CLIENT_LIBRARY_REVERSE_INTERFACE_H_INCLUDED

#include "ur_client_library/comm/tcp_server.h"
#include "ur_client_library/comm/control_mode.h"
#include "ur_client_library/types.h"
#include "ur_client_library/log.h"
#include "ur_client_library/ur/robot_receive_timeout.h"
#include <math.h>
#include <cstring>
#include <cassert>
#include <endian.h>
#include <condition_variable>
#include <optional>

namespace urcl
{
namespace control
{
/*!
 * \brief Control messages for forwarding and aborting trajectories.
 */
enum class TrajectoryControlMessage : int32_t
{
  TRAJECTORY_CANCEL = -1,  ///< Represents command to cancel currently active trajectory.
  TRAJECTORY_NOOP = 0,     ///< Represents no new control command.
  TRAJECTORY_START = 1,    ///< Represents command to start a new trajectory.
};

/*!
 * \brief Control messages for starting and stopping freedrive mode.
 */
enum class FreedriveControlMessage : int32_t
{
  FREEDRIVE_STOP = -1,  ///< Represents command to stop freedrive mode.
  FREEDRIVE_NOOP = 0,   ///< Represents keep running in freedrive mode.
  FREEDRIVE_START = 1,  ///< Represents command to start freedrive mode.
};

/*!
 * \brief The ReverseInterface class handles communication to the robot. It starts a server and
 * waits for the robot to connect via its URCaps program.
 */
class ReverseInterface
{
public:
  static const int32_t MULT_JOINTSTATE = 1000000;

  /*!
  * \brief Container for binary list of enabling/disabling axes
  */
  class BinaryArray
  {
  public:
    /// @brief BinaryArray constructor
    /// @note The default configuration of this constructor is for all axes to be active
    /// @param[in] vec boolean array of length=6
    BinaryArray(const std::array<bool, 6>& vec = {true, true, true, true, true, true}) : vec_(vec) {};
    BinaryArray(const vector6d_t& vec) : 
    vec_(
      {vec[0] == 1.0 ? true : false, 
      vec[1] == 1.0 ? true : false,
      vec[2] == 1.0 ? true : false,
      vec[3] == 1.0 ? true : false,
      vec[4] == 1.0 ? true : false,
      vec[5] == 1.0 ? true : false}
    ) {};
    
    /// @brief Output s32 buffer for passing to the TCP server. s32 is a binary list meant to be
    /// passed to the urscript function `integer_to_binary_list`.
    /// @return s32, with the joint axis at index 0 starting at the LSB.
    int32_t ToS32Buffer() const {
      int32_t out = 0;
      for (size_t i = 0; i < 6; ++i)
      {
        // Stuff 0 or 1 at index i to activate/deactivate axis 
        out = (out | (vec_[i] << i));
      }
      
      return out;
    };

    size_t GetBufferInt32Length() const {
      return 1;
    }
  private:
    const std::array<bool, 6> vec_;
  };
  
  /*!
  * \brief Container for freedrive feature
  */
  class Feature
  {
  public:
    static constexpr int32_t feature_literal_custom = 0;

    /*!
    * \brief Container for feature literals for .
    */
    enum class FeatureLiterals : int32_t {
      /// 0 is reserved to indicate that a custom vector is being passed.
      BASE = 1,   ///< Indicates that the frame of reference is the "base"
      TOOL = 2    ///< Indicates that the frame of reference is the "tool"
    };
    
    /// @brief Feature constructor with pose vector
    /// @note The default configuration of this constructor is consistent with the default 
    /// freedrive `feature` variable
    /// @param[in] vec double array of length=6
    Feature(const vector6d_t& vec = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}) : vec_(vec), name_(std::nullopt) {};
    
    /// @brief Feature constructor with feature literal
    /// @param[in] feature_name Feature name
    Feature(FeatureLiterals feature_name) : vec_(std::nullopt), name_(feature_name) {};
    
    /// @brief Output s32 buffer for passing to the TCP server.
    /// @note Buffer is dynamically allocated and must be freed!!!
    /// @param[out] bufsize Optional, pointer to an s32 to output the size of allocated buffer
    /// @return Pointer to s32 buffer of length `bufsize`
    int32_t *ToS32Buffer(size_t *bufsize) const {
      // Static assert that our bufsize is not too big
      // TODO(george): should there be a declaration elsewhere for the 4 (represents # of additional dwords required)
      static_assert(bufsize_ <= MAX_MESSAGE_LENGTH - 4, "Feature buffer is too large");

      // Allocate buffer
      int32_t *out = (int32_t *)std::malloc(sizeof(int32_t) * bufsize_);
      assert(out != nullptr && "Unable to allocate buffer");

      // Set memory block to zero
      std::memset((void *)out, 0, sizeof(int32_t) * bufsize_);

      if (vec_.has_value())
      {
        // Case where a vector has was suplied
        for (size_t i = 0; i < bufsize_-2; ++i)
        {
          // Simply copy all elements to the buffer. 
          // Convert to s32 w/ MULT_JOINTSTATE factor.
          out[i] = htobe32(static_cast<int32_t>(round(vec_.value()[i] * MULT_JOINTSTATE)));
        }

        // Set last element meant to store literal string info
        out[bufsize_-1] = htobe32(feature_literal_custom);
      }
      else if (name_.has_value())
      {
        // Set last element to the identifier to pass
        // Other elements can be undefined since they will be ignored
        out[bufsize_-1] = htobe32(toUnderlying(name_.value()));
      }
      else
      {
        std::free(out); // TODO(george): needed?
        assert(false && "Undefined behavior");
      }

      if (bufsize != NULL)
      {
        *bufsize = bufsize_ * sizeof(int32_t);
      }

      return out;
    };

    size_t GetBufferInt32Length() const {
      return bufsize_;
    }
  private:
    static constexpr size_t bufsize_ = 7;
    const std::optional<vector6d_t> vec_;
    const std::optional<FeatureLiterals> name_;
  };

  ReverseInterface() = delete;
  /*!
   * \brief Creates a ReverseInterface object including a TCPServer.
   *
   * \param port Port the Server is started on
   * \param handle_program_state Function handle to a callback on program state changes.
   * \param step_time The robots step time
   */
  ReverseInterface(uint32_t port, std::function<void(bool)> handle_program_state,
                   std::chrono::milliseconds step_time = std::chrono::milliseconds(8));

  /*!
   * \brief Disconnects possible clients so the reverse interface object can be safely destroyed.
   */
  virtual ~ReverseInterface() = default;

  /*!
   * \brief Writes needed information to the robot to be read by the URCaps program.
   *
   * \param positions A vector of joint targets for the robot
   * \param control_mode Control mode assigned to this command. See documentation of comm::ControlMode
   * for details on possible values.
   * \param robot_receive_timeout The read timeout configuration for the reverse socket running in the external
   * control script on the robot. Use with caution when dealing with realtime commands as the robot
   * expects to get a new control signal each control cycle. Note the timeout cannot be higher than 1 second for
   * realtime commands.
   *
   * \returns True, if the write was performed successfully, false otherwise.
   */
  virtual bool write(const vector6d_t* positions, const comm::ControlMode control_mode = comm::ControlMode::MODE_IDLE,
                     const RobotReceiveTimeout& robot_receive_timeout = RobotReceiveTimeout::millisec(20));

  /*!
   * \brief Writes needed information to the robot to be read by the URScript program.
   *
   * \param trajectory_action 1 if a trajectory is to be started, -1 if it should be stopped
   * \param point_number The number of points of the trajectory to be executed
   * \param robot_receive_timeout The read timeout configuration for the reverse socket running in the external
   * control script on the robot. If you want to make the read function blocking then use RobotReceiveTimeout::off()
   * function to create the RobotReceiveTimeout object
   *
   * \returns True, if the write was performed successfully, false otherwise.
   */
  bool
  writeTrajectoryControlMessage(const TrajectoryControlMessage trajectory_action, const int point_number = 0,
                                const RobotReceiveTimeout& robot_receive_timeout = RobotReceiveTimeout::millisec(200));

  /*!
   * \brief Writes needed information to the robot to be read by the URScript program.
   *
   * \param freedrive_action 1 if freedrive mode is to be started, -1 if it should be stopped and 0 to keep it running
   * \param robot_receive_timeout The read timeout configuration for the reverse socket running in the external
   * control script on the robot. If you want to make the read function blocking then use RobotReceiveTimeout::off()
   * function to create the RobotReceiveTimeout object
   * \param free_axes See UR script manual for full description.
   * \param feature See UR script manual for full description.
   *
   * \returns True, if the write was performed successfully, false otherwise.
   */
  bool
  writeFreedriveControlMessage(const FreedriveControlMessage freedrive_action,
                               const RobotReceiveTimeout& robot_receive_timeout = RobotReceiveTimeout::millisec(200),
                               const BinaryArray& free_axes = BinaryArray(),
                               const Feature& feature = Feature());

  bool 
  writeDynamicForceModeMessage(const vector6d_t& task_frame,
                               const BinaryArray& compliance_vector,
                               const vector6d_t& wrench,
                               const RobotReceiveTimeout& robot_receive_timeout);


  /*!
   * \brief Set the Keepalive count. This will set the number of allowed timeout reads on the robot.
   *
   * \param count Number of allowed timeout reads on the robot.
   */
  [[deprecated("Set keepaliveCount is deprecated, instead use the robot receive timeout directly in the write "
               "commands.")]] virtual void
  setKeepaliveCount(const uint32_t count);

protected:
  virtual void connectionCallback(const int filedescriptor);

  virtual void disconnectionCallback(const int filedescriptor);

  virtual void messageCallback(const int filedescriptor, char* buffer, int nbytesrecv);

  int client_fd_;
  comm::TCPServer server_;

  template <typename T>
  size_t append(uint8_t* buffer, T& val)
  {
    size_t s = sizeof(T);
    std::memcpy(buffer, &val, s);
    return s;
  }

  static const int MAX_MESSAGE_LENGTH = 15;

  std::function<void(bool)> handle_program_state_;
  std::chrono::milliseconds step_time_;

  uint32_t keepalive_count_;
  bool keep_alive_count_modified_deprecated_;
};

}  // namespace control
}  // namespace urcl

#endif  // UR_CLIENT_LIBRARY_REVERSE_INTERFACE_H_INCLUDED
