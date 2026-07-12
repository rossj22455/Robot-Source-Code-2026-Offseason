// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

/** A convex or concave polygonal region of the field, defined by its vertices in order. */
public class Region2d {
  private final Translation2d[] vertices;

  public Region2d(Translation2d[] vertices) {
    if (vertices.length < 3) {
      throw new IllegalArgumentException("Region2d requires at least 3 vertices");
    }
    this.vertices = vertices.clone();
  }

  /** Returns true if the point is inside this region (even-odd ray casting). */
  public boolean contains(Translation2d point) {
    boolean inside = false;
    for (int i = 0, j = vertices.length - 1; i < vertices.length; j = i++) {
      double xi = vertices[i].getX();
      double yi = vertices[i].getY();
      double xj = vertices[j].getX();
      double yj = vertices[j].getY();
      if ((yi > point.getY()) != (yj > point.getY())
          && point.getX() < (xj - xi) * (point.getY() - yi) / (yj - yi) + xi) {
        inside = !inside;
      }
    }
    return inside;
  }
}
