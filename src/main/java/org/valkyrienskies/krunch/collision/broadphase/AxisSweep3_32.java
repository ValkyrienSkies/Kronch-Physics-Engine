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
 * AxisSweep3_32 allows higher precision quantization and more objects compared to the {@link AxisSweep3} sweep and
 * prune. This comes at the cost of more memory per handle, and a bit slower performance.
 *
 * @author jezek2
 */
public class AxisSweep3_32 extends AxisSweep3Internal {

    public AxisSweep3_32(final Vector3f worldAabbMin, final Vector3f worldAabbMax) {
        this(worldAabbMin, worldAabbMax, 1500000, null);
    }

    public AxisSweep3_32(final Vector3f worldAabbMin, final Vector3f worldAabbMax, final int maxHandles) {
        this(worldAabbMin, worldAabbMax, maxHandles, null);
    }

    public AxisSweep3_32(final Vector3f worldAabbMin, final Vector3f worldAabbMax, final int maxHandles/* = 1500000*/,
        final OverlappingPairCache pairCache/* = 0*/) {
        super(worldAabbMin, worldAabbMax, 0xfffffffe, 0x7fffffff, maxHandles, pairCache);
        // 1 handle is reserved as sentinel
        assert (maxHandles > 1 && maxHandles < 2147483647);
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
        return 0xFFFFFFFF;
    }

    protected static class EdgeArrayImpl extends EdgeArray {
        private int[] pos;
        private int[] handle;

        public EdgeArrayImpl(final int size) {
            pos = new int[size];
            handle = new int[size];
        }

        @Override
        public void swap(final int idx1, final int idx2) {
            final int tmpPos = pos[idx1];
            final int tmpHandle = handle[idx1];

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
            return pos[index];
        }

        @Override
        public void setPos(final int index, final int value) {
            pos[index] = value;
        }

        @Override
        public int getHandle(final int index) {
            return handle[index];
        }

        @Override
        public void setHandle(final int index, final int value) {
            handle[index] = value;
        }
    }

    protected static class HandleImpl extends Handle {
        private int minEdges0;
        private int minEdges1;
        private int minEdges2;

        private int maxEdges0;
        private int maxEdges1;
        private int maxEdges2;

        @Override
        public int getMinEdges(final int edgeIndex) {
            switch (edgeIndex) {
                default:
                case 0:
                    return minEdges0;
                case 1:
                    return minEdges1;
                case 2:
                    return minEdges2;
            }
        }

        @Override
        public void setMinEdges(final int edgeIndex, final int value) {
            switch (edgeIndex) {
                case 0:
                    minEdges0 = value;
                    break;
                case 1:
                    minEdges1 = value;
                    break;
                case 2:
                    minEdges2 = value;
                    break;
            }
        }

        @Override
        public int getMaxEdges(final int edgeIndex) {
            switch (edgeIndex) {
                default:
                case 0:
                    return maxEdges0;
                case 1:
                    return maxEdges1;
                case 2:
                    return maxEdges2;
            }
        }

        @Override
        public void setMaxEdges(final int edgeIndex, final int value) {
            switch (edgeIndex) {
                case 0:
                    maxEdges0 = value;
                    break;
                case 1:
                    maxEdges1 = value;
                    break;
                case 2:
                    maxEdges2 = value;
                    break;
            }
        }
    }

}
