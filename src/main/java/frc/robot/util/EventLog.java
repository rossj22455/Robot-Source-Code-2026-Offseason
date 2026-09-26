// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import org.littletonrobotics.junction.Logger;

/**
 * Timestamped event log for post-match review. Records a short message under the "Events" log key
 * only when something happens (command start/end/interrupt, brownout, comms, camera disconnect,
 * homing result), so it costs nothing in the normal loop and never prints to the console. In
 * AdvantageScope, add "Events" as a text field and scrub the timeline to read what happened when.
 */
public final class EventLog {
  private static int count = 0;

  private EventLog() {}

  /** Records one event. Numbered so identical back-to-back events stay distinguishable. */
  public static void log(String message) {
    count++;
    Logger.recordOutput("Events", "#" + count + " " + message);
  }
}
