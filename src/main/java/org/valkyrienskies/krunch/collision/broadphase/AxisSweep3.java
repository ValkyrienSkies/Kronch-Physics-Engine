/*
 * Java port of Bullet (c) 2008 Martin Dvorak <jezek2@advel.cz>
 *
 * AxisSweep3
 * Copyright (c) 2006 Simon Hobbs
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
 * AxisSweep3 is an efficient implementation of the 3D axis sweep and prune broadphase.<p>
 * <p>
 * It uses arrays rather then lists for storage of the 3 axis. Also it operates using 16 bit integer coordinates instead
 * of floats. For large worlds and many objects, use {@link AxisSweep3_32} instead. AxisSweep3_32 has higher precision
 * and allows more than 16384 objects at the cost of more memory and bit of performance.
 *
 * @author jezek2
 */
public class AxisSweep3 extends AxisSweep3Internal {

    public AxisSweep3(final Vector3f worldAabbMin, final Vector3f worldAabbMax) {
        this(worldAabbMin, worldAabbMax, 16384, null);
    }

    public AxisSweep3(final Vector3f worldAabbMin, final Vector3f worldAabbMax, final int maxHandles) {
        this(worldAabbMin, worldAabbMax, maxHandles, null);
    }

    public AxisSweep3(final Vector3f worldAabbMin, final Vector3f worldAabbMax, final int maxHandles/* = 16384*/,
        final OverlappingPairCache pairCache/* = 0*/) {
        super(worldAabbMin, worldAabbMax, 0xfffe, 0xffff, maxHandles, pairCache);
        // 1 handle is reserved as sentinel
        assert (maxHandles > 1 && maxHandles < 32767);
    }

    @Override
    protected EdgeArray createEdgeArray(final int size) {
        return new EdgeArrayImpl(size);
    }

    @Override
    protected Handle createHandle() {
        return new HandleImpl();
    }

    protected int getMask() {
        return 0xFFFF;
    }

    protected static class EdgeArrayImpl extends EdgeArray {
        private short[] pos;
        private short[] handle;

        public EdgeArrayImpl(final int size) {
            pos = new short[size];
            handle = new short[size];
        }

        @Override
        public void swap(final int idx1, final int idx2) {
            final short tmpPos = pos[idx1];
            final short tmpHandle = handle[idx1];

            pos[idx1] = pos[idx2];
            handle[idx1] = handle[idx2];

            pos[idx2] = tmpPos;
            handle[idx2] = tmpHandle;
        }

        @Override
        public void set(final int dest, final int src) {
            pos[dest] = pos[src];
            handle[dest] = handle[src];
        }

        @Override
        public int getPos(final int index) {
            return pos[index] & 0xFFFF;
        }

        @Override
        public void setPos(final int index, final int value) {
            pos[index] = (short) value;
        }

        @Override
        public int getHandle(final int index) {
            return handle[index] & 0xFFFF;
        }

        @Override
        public void setHandle(final int index, final int value) {
            handle[index] = (short) value;
        }
    }

    protected static class HandleImpl extends Handle {
        private short minEdges0;
        private short minEdges1;
        private short minEdges2;

        private short maxEdges0;
        private short maxEdges1;
        private short maxEdges2;

        @Override
        public int getMinEdges(final int edgeIndex) {
            switch (edgeIndex) {
                default:
                case 0:
                    return minEdges0 & 0xFFFF;
                case 1:
                    return minEdges1 & 0xFFFF;
                case 2:
                    return minEdges2 & 0xFFFF;
            }
        }

        @Override
        public void setMinEdges(final int edgeIndex, final int value) {
            switch (edgeIndex) {
                case 0:
                    minEdges0 = (short) value;
                    break;
                case 1:
                    minEdges1 = (short) value;
                    break;
                case 2:
                    minEdges2 = (short) value;
                    break;
            }
        }

        @Override
        public int getMaxEdges(final int edgeIndex) {
            switch (edgeIndex) {
                default:
                case 0:
                    return maxEdges0 & 0xFFFF;
                case 1:
                    return maxEdges1 & 0xFFFF;
                case 2:
                    return maxEdges2 & 0xFFFF;
            }
        }

        @Override
        public void setMaxEdges(final int edgeIndex, final int value) {
            switch (edgeIndex) {
                case 0:
                    maxEdges0 = (short) value;
                    break;
                case 1:
                    maxEdges1 = (short) value;
                    break;
                case 2:
                    maxEdges2 = (short) value;
                    break;
            }
        }
    }

}
