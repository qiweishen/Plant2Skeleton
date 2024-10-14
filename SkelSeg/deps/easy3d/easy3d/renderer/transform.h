/********************************************************************
 * Copyright (C) 2015 Liangliang Nan <liangliang.nan@gmail.com>
 * https://3d.bk.tudelft.nl/liangliang/
 *
 * This file is part of Easy3D. If it is useful in your research/work,
 * I would be grateful if you show your appreciation by citing it:
 * ------------------------------------------------------------------
 *      Liangliang Nan.
 *      Easy3D: a lightweight, easy-to-use, and efficient C++ library
 *      for processing and rendering 3D data.
 *      Journal of Open Source Software, 6(64), 3255, 2021.
 * ------------------------------------------------------------------
 *
 * Easy3D is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License Version 3
 * as published by the Free Software Foundation.
 *
 * Easy3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 ********************************************************************/

#ifndef EASY3D_RENDERER_TRANSFORM_H
#define EASY3D_RENDERER_TRANSFORM_H

#include <easy3d/core/types.h>


namespace easy3d {

    /**
     * Defines functions that generate common transformation matrices (all using the right-handed coordinate system).
     * The matrices generated by this extension use standard OpenGL fixed-function conventions.
     * For example, the lookAt function generates a transform from world space into the specific eye space that
     * the projective matrix functions (perspective, ortho, etc.) are designed to expect. The OpenGL compatibility
     * specifications defines the particular layout of this eye space.
     */
    namespace transform {

        /**
         * Creates a matrix for an orthographic parallel viewing volume. Simulating glFrustum().
         * \param near Specifies the distance from the viewer to the near clipping plane (always positive).
         * \param far  Specifies the distance from the viewer to the far clipping plane (always positive).
         * See http://www.songho.ca/opengl/gl_projectionmatrix.html
         *     https://ksimek.github.io/2013/06/03/calibrated_cameras_in_opengl/
         */
        mat4 ortho(float left, float right, float bottom, float top, float near, float far);

        /// Creates a matrix for projecting two-dimensional coordinates onto the screen.
        mat4 ortho(float left, float right, float bottom, float top);

        /**
         * Creates a frustum perspective matrix. Simulating glFrustum().
         * See http://www.songho.ca/opengl/gl_projectionmatrix.html
         * 	   https://ksimek.github.io/2013/06/03/calibrated_cameras_in_opengl/
         */
        mat4 frustum(float left, float right, float bottom, float top, float near, float far);

        /**
         * Creates a matrix for a right-handed symmetric perspective-view frustum. Simulating gluPerspective().
         * \param fov_y  Specifies the field of view angle, in the y direction. Expressed in radians.
         * \param aspect Specifies the aspect ratio that determines the field of view in the x direction. The aspect
         *      ratio is the ratio of x (width) to y (height).
         * \param near   Specifies the distance from the viewer to the near clipping plane (always positive).
         * \param far    Specifies the distance from the viewer to the far clipping plane (always positive).
         * \note Degrees are an unhandy unit to work with. Thus, I use radians for everything!
         * See https://ksimek.github.io/2013/06/18/calibrated-cameras-and-gluperspective/
         */
        mat4 perspective(float fov_y, float aspect, float near, float far);
        mat4 perspective(float fov_y, float width, float height, float near, float far);

        /**
         * Creates a matrix for a symmetric perspective-view frustum with far plane at infinite.
         * \param fov_y  Specifies the field of view angle, in the y direction. Expressed in radians.
         * \param aspect Specifies the aspect ratio that determines the field of view in the x direction. The
         *		   aspect ratio is the ratio of x (width) to y (height).
         * \param near   Specifies the distance from the viewer to the near clipping plane (always positive).
         */
        mat4 infinite_perspective(float fov_y, float aspect, float near);

        /// Creates a viewport matrix. Simulating glViewport().
        mat4 viewport(float width, float height);

        /**
         * Builds a look at view matrix simulating gluLookAt().
         * \param eye    Position of the camera.
         * \param center Position where the camera is looking at.
         * \param up     Normalized up vector determining how the camera is oriented. Typically (0, 0, 1).
         */
        mat4 look_at(const vec3& eye, const vec3& center, const vec3& up);

        /// Defines a picking region
        mat4 pick_matrix(const vec2& center, const vec2& delta, const vec4& viewport);

        /**
         * Maps the specified object coordinates (obj.x, obj.y, obj.z) into window coordinates. Simulating gluProject().
         * \param obj   Specifies the object coordinates.
         * \param model Specifies the current model-view matrix.
         * \param proj  Specifies the current projection matrix.
         * \param viewport Specifies the current viewport.
         *				viewport[0] = 0;
         *				viewport[1] = 0;
         *				viewport[2] = screenWidth();
         *				viewport[3] = screenHeight();
         * \note OpenGL uses the lower corner for its origin while other software (e.g., Qt) may use upper corner.
         * \return The computed window coordinates.
         */
        vec3 project(const vec3& obj, const mat4& model, const mat4& proj, const int viewport[4], bool lowerleft = true);
        vec3 project(const vec3& obj, const mat4& mvp, const int viewport[4], bool lowerleft = true); // mvp = proj * model;

        /**
         * Maps the specified window coordinates (win.x, win.y, win.z) into object coordinates. Simulating gluUnProject().
         * \param win   Specifies the window coordinates to be mapped.
         * \param model Specifies the model-view matrix.
         * \param proj  Specifies the projection matrix.
         * \param viewport Specifies the viewport.
         *				viewport[0] = 0;
         *				viewport[1] = 0;
         *				viewport[2] = screenWidth();
         *				viewport[3] = screenHeight();
         * \note OpenGL uses the lower corner for its origin while other software (e.g., Qt) may use upper corner.
         * \return The computed object coordinates.
         */
        vec3 unproject(const vec3& win, const mat4& model, const mat4& proj, const int viewport[4], bool lowerleft = true);
        vec3 unproject(const vec3& win, const mat4& mvp, const int viewport[4], bool lowerleft = true); // mvp = proj * model;

        // --------------------------------------------------------------------------------------------------

        /**
         * Computes the normal matrix based on mat.
         * \note The returned matrix is NOT padded. Use the padded version for uniform blocks.
         */
        mat3  normal_matrix(const mat4& mat);

        /**
         * Computes the normal matrix based on mat.
         * \note This is the padded version suitable for uniform blocks.
         */
        mat43 normal_matrix_padded(const mat4& mat);

    } // namespace transform

}


#endif  // EASY3D_RENDERER_TRANSFORM_H