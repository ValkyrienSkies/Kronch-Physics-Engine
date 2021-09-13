/*
 * Java port of Bullet (c) 2008 Martin Dvorak <jezek2@advel.cz>
 *
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2008 Erwin Coumans  http://www.bulletphysics.com/
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

package org.valkyrienskies.krunch.collision.broadphase;

import org.joml.Vector3f;

/**
 * Utility functions for vectors.
 *
 * @author jezek2
 */
public class VectorUtil {

    public static int maxAxis(final Vector3f v) {
        int maxIndex = -1;
        float maxVal = -1e30f;
        if (v.x > maxVal) {
            maxIndex = 0;
            maxVal = v.x;
        }
        if (v.y > maxVal) {
            maxIndex = 1;
            maxVal = v.y;
        }
        if (v.z > maxVal) {
            maxIndex = 2;
            maxVal = v.z;
        }

        return maxIndex;
    }

    public static float getCoord(final Vector3f vec, final int num) {
        switch (num) {
            case 0:
                return vec.x;
            case 1:
                return vec.y;
            case 2:
                return vec.z;
            default:
                throw new InternalError();
        }
    }

    public static void setCoord(final Vector3f vec, final int num, final float value) {
        switch (num) {
            case 0:
                vec.x = value;
                break;
            case 1:
                vec.y = value;
                break;
            case 2:
                vec.z = value;
                break;
            default:
                throw new InternalError();
        }
    }

    public static void mulCoord(final Vector3f vec, final int num, final float value) {
        switch (num) {
            case 0:
                vec.x *= value;
                break;
            case 1:
                vec.y *= value;
                break;
            case 2:
                vec.z *= value;
                break;
            default:
                throw new InternalError();
        }
    }

    public static void setInterpolate3(final Vector3f dest, final Vector3f v0, final Vector3f v1, final float rt) {
        final float s = 1f - rt;
        dest.x = s * v0.x + rt * v1.x;
        dest.y = s * v0.y + rt * v1.y;
        dest.z = s * v0.z + rt * v1.z;
        // don't do the unused w component
        //		m_co[3] = s * v0[3] + rt * v1[3];
    }

    public static void add(final Vector3f dest, final Vector3f v1, final Vector3f v2) {
        dest.x = v1.x + v2.x;
        dest.y = v1.y + v2.y;
        dest.z = v1.z + v2.z;
    }

    public static void add(final Vector3f dest, final Vector3f v1, final Vector3f v2, final Vector3f v3) {
        dest.x = v1.x + v2.x + v3.x;
        dest.y = v1.y + v2.y + v3.y;
        dest.z = v1.z + v2.z + v3.z;
    }

    public static void add(final Vector3f dest, final Vector3f v1, final Vector3f v2, final Vector3f v3,
        final Vector3f v4) {
        dest.x = v1.x + v2.x + v3.x + v4.x;
        dest.y = v1.y + v2.y + v3.y + v4.y;
        dest.z = v1.z + v2.z + v3.z + v4.z;
    }

    public static void mul(final Vector3f dest, final Vector3f v1, final Vector3f v2) {
        dest.x = v1.x * v2.x;
        dest.y = v1.y * v2.y;
        dest.z = v1.z * v2.z;
    }

    public static void div(final Vector3f dest, final Vector3f v1, final Vector3f v2) {
        dest.x = v1.x / v2.x;
        dest.y = v1.y / v2.y;
        dest.z = v1.z / v2.z;
    }

    public static void setMin(final Vector3f a, final Vector3f b) {
        a.x = Math.min(a.x, b.x);
        a.y = Math.min(a.y, b.y);
        a.z = Math.min(a.z, b.z);
    }

    public static void setMax(final Vector3f a, final Vector3f b) {
        a.x = Math.max(a.x, b.x);
        a.y = Math.max(a.y, b.y);
        a.z = Math.max(a.z, b.z);
    }

}
