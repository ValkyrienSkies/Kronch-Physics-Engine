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

// Dbvt implementation by Nathanael Presson

package org.valkyrienskies.krunch.collision.broadphase;

import it.unimi.dsi.fastutil.ints.IntArrayList;
import java.util.Collections;
import org.joml.Vector3f;

/**
 * @author jezek2
 */
public class Dbvt {

    public static final int SIMPLE_STACKSIZE = 64;
    public static final int DOUBLE_STACKSIZE = SIMPLE_STACKSIZE * 2;

    public Node root = null;
    public Node free = null;
    public int lkhd = -1;
    public int leaves = 0;
    public /*unsigned*/ int opath = 0;

    public Dbvt() {
    }

    public void clear() {
        if (root != null) {
            recursedeletenode(this, root);
        }
        //btAlignedFree(m_free);
        free = null;
    }

    public boolean empty() {
        return (root == null);
    }

    public void optimizeBottomUp() {
        if (root != null) {
            final ObjectArrayList<Node> leaves = new ObjectArrayList<>(this.leaves);
            fetchleaves(this, root, leaves);
            bottomup(this, leaves);
            root = leaves.getQuick(0);
        }
    }

    public void optimizeTopDown() {
        optimizeTopDown(128);
    }

    public void optimizeTopDown(final int bu_treshold) {
        if (root != null) {
            final ObjectArrayList<Node> leaves = new ObjectArrayList<>(this.leaves);
            fetchleaves(this, root, leaves);
            root = topdown(this, leaves, bu_treshold);
        }
    }

    public void optimizeIncremental(int passes) {
        if (passes < 0) {
            passes = leaves;
        }

        if (root != null && (passes > 0)) {
            final Node[] root_ref = new Node[1];
            do {
                Node node = root;
                int bit = 0;
                while (node.isinternal()) {
                    root_ref[0] = root;
                    node = sort(node, root_ref).childs[(opath >>> bit) & 1];
                    root = root_ref[0];

                    bit = (bit + 1) & (/*sizeof(unsigned)*/4 * 8 - 1);
                }
                update(node);
                ++opath;
            }
            while ((--passes) != 0);
        }
    }

    public Node insert(final DbvtAabbMm box, final Object data) {
        final Node leaf = createnode(this, null, box, data);
        insertleaf(this, root, leaf);
        leaves++;
        return leaf;
    }

    public void update(final Node leaf) {
        update(leaf, -1);
    }

    public void update(final Node leaf, final int lookahead) {
        Node root = removeleaf(this, leaf);
        if (root != null) {
            if (lookahead >= 0) {
                for (int i = 0; (i < lookahead) && root.parent != null; i++) {
                    root = root.parent;
                }
            } else {
                root = this.root;
            }
        }
        insertleaf(this, root, leaf);
    }

    public void update(final Node leaf, final DbvtAabbMm volume) {
        Node root = removeleaf(this, leaf);
        if (root != null) {
            if (lkhd >= 0) {
                for (int i = 0; (i < lkhd) && root.parent != null; i++) {
                    root = root.parent;
                }
            } else {
                root = this.root;
            }
        }
        leaf.volume.set(volume);
        insertleaf(this, root, leaf);
    }

    public boolean update(final Node leaf, final DbvtAabbMm volume, final Vector3f velocity, final float margin) {
        if (leaf.volume.Contain(volume)) {
            return false;
        }
        final Vector3f tmp = new Vector3f(); // Stack.alloc(Vector3f.class);
        tmp.set(margin, margin, margin);
        volume.Expand(tmp);
        volume.SignedExpand(velocity);
        update(leaf, volume);
        return true;
    }

    public boolean update(final Node leaf, final DbvtAabbMm volume, final Vector3f velocity) {
        if (leaf.volume.Contain(volume)) {
            return false;
        }
        volume.SignedExpand(velocity);
        update(leaf, volume);
        return true;
    }

    public boolean update(final Node leaf, final DbvtAabbMm volume, final float margin) {
        if (leaf.volume.Contain(volume)) {
            return false;
        }
        final Vector3f tmp = new Vector3f(); // Stack.alloc(Vector3f.class);
        tmp.set(margin, margin, margin);
        volume.Expand(tmp);
        update(leaf, volume);
        return true;
    }

    public void remove(final Node leaf) {
        removeleaf(this, leaf);
        deletenode(this, leaf);
        leaves--;
    }

    public void write(final IWriter iwriter) {
        throw new UnsupportedOperationException();
    }

    public void clone(final Dbvt dest) {
        clone(dest, null);
    }

    public void clone(final Dbvt dest, final IClone iclone) {
        throw new UnsupportedOperationException();
    }

    public static int countLeaves(final Node node) {
        if (node.isinternal()) {
            return countLeaves(node.childs[0]) + countLeaves(node.childs[1]);
        } else {
            return 1;
        }
    }

    public static void extractLeaves(final Node node, final ObjectArrayList<Node> leaves) {
        if (node.isinternal()) {
            extractLeaves(node.childs[0], leaves);
            extractLeaves(node.childs[1], leaves);
        } else {
            leaves.add(node);
        }
    }

    public static void enumNodes(final Node root, final ICollide policy) {
        //DBVT_CHECKTYPE
        policy.Process(root);
        if (root.isinternal()) {
            enumNodes(root.childs[0], policy);
            enumNodes(root.childs[1], policy);
        }
    }

    public static void enumLeaves(final Node root, final ICollide policy) {
        //DBVT_CHECKTYPE
        if (root.isinternal()) {
            enumLeaves(root.childs[0], policy);
            enumLeaves(root.childs[1], policy);
        } else {
            policy.Process(root);
        }
    }

    public static void collideTT(final Node root0, final Node root1, final ICollide policy) {
        //DBVT_CHECKTYPE
        if (root0 != null && root1 != null) {
            final ObjectArrayList<sStkNN> stack = new ObjectArrayList<>(DOUBLE_STACKSIZE);
            stack.add(new sStkNN(root0, root1));
            do {
                final sStkNN p = stack.remove(stack.size() - 1);
                if (p.a == p.b) {
                    if (p.a.isinternal()) {
                        stack.add(new sStkNN(p.a.childs[0], p.a.childs[0]));
                        stack.add(new sStkNN(p.a.childs[1], p.a.childs[1]));
                        stack.add(new sStkNN(p.a.childs[0], p.a.childs[1]));
                    }
                } else if (DbvtAabbMm.Intersect(p.a.volume, p.b.volume)) {
                    if (p.a.isinternal()) {
                        if (p.b.isinternal()) {
                            stack.add(new sStkNN(p.a.childs[0], p.b.childs[0]));
                            stack.add(new sStkNN(p.a.childs[1], p.b.childs[0]));
                            stack.add(new sStkNN(p.a.childs[0], p.b.childs[1]));
                            stack.add(new sStkNN(p.a.childs[1], p.b.childs[1]));
                        } else {
                            stack.add(new sStkNN(p.a.childs[0], p.b));
                            stack.add(new sStkNN(p.a.childs[1], p.b));
                        }
                    } else {
                        if (p.b.isinternal()) {
                            stack.add(new sStkNN(p.a, p.b.childs[0]));
                            stack.add(new sStkNN(p.a, p.b.childs[1]));
                        } else {
                            policy.Process(p.a, p.b);
                        }
                    }
                }
            }
            while (stack.size() > 0);
        }
    }

    public static void collideTT(final Node root0, final Node root1, final Transform xform, final ICollide policy) {
        //DBVT_CHECKTYPE
        if (root0 != null && root1 != null) {
            final ObjectArrayList<sStkNN> stack = new ObjectArrayList<>(DOUBLE_STACKSIZE);
            stack.add(new sStkNN(root0, root1));
            do {
                final sStkNN p = stack.remove(stack.size() - 1);
                if (p.a == p.b) {
                    if (p.a.isinternal()) {
                        stack.add(new sStkNN(p.a.childs[0], p.a.childs[0]));
                        stack.add(new sStkNN(p.a.childs[1], p.a.childs[1]));
                        stack.add(new sStkNN(p.a.childs[0], p.a.childs[1]));
                    }
                } else if (DbvtAabbMm.Intersect(p.a.volume, p.b.volume, xform)) {
                    if (p.a.isinternal()) {
                        if (p.b.isinternal()) {
                            stack.add(new sStkNN(p.a.childs[0], p.b.childs[0]));
                            stack.add(new sStkNN(p.a.childs[1], p.b.childs[0]));
                            stack.add(new sStkNN(p.a.childs[0], p.b.childs[1]));
                            stack.add(new sStkNN(p.a.childs[1], p.b.childs[1]));
                        } else {
                            stack.add(new sStkNN(p.a.childs[0], p.b));
                            stack.add(new sStkNN(p.a.childs[1], p.b));
                        }
                    } else {
                        if (p.b.isinternal()) {
                            stack.add(new sStkNN(p.a, p.b.childs[0]));
                            stack.add(new sStkNN(p.a, p.b.childs[1]));
                        } else {
                            policy.Process(p.a, p.b);
                        }
                    }
                }
            }
            while (stack.size() > 0);
        }
    }

    public static void collideTT(final Node root0, final Transform xform0, final Node root1, final Transform xform1,
        final ICollide policy) {
        final Transform xform = new Transform(); // Stack.alloc(Transform.class);
        xform.inverse(xform0);
        xform.mul(xform1);
        collideTT(root0, root1, xform, policy);
    }

    public static void collideTV(final Node root, final DbvtAabbMm volume, final ICollide policy) {
        //DBVT_CHECKTYPE
        if (root != null) {
            final ObjectArrayList<Node> stack = new ObjectArrayList<>(SIMPLE_STACKSIZE);
            stack.add(root);
            do {
                final Node n = stack.remove(stack.size() - 1);
                if (DbvtAabbMm.Intersect(n.volume, volume)) {
                    if (n.isinternal()) {
                        stack.add(n.childs[0]);
                        stack.add(n.childs[1]);
                    } else {
                        policy.Process(n);
                    }
                }
            }
            while (stack.size() > 0);
        }
    }

    public static void collideRAY(final Node root, final Vector3f origin, final Vector3f direction,
        final ICollide policy) {
        //DBVT_CHECKTYPE
        if (root != null) {
            final Vector3f normal = new Vector3f(); // Stack.alloc(Vector3f.class);
            normal.normalize(direction);
            final Vector3f invdir = new Vector3f(); // Stack.alloc(Vector3f.class);
            invdir.set(1f / normal.x, 1f / normal.y, 1f / normal.z);
            final int[] signs = new int[] {direction.x < 0 ? 1 : 0, direction.y < 0 ? 1 : 0, direction.z < 0 ? 1 : 0};
            final ObjectArrayList<Node> stack = new ObjectArrayList<>(SIMPLE_STACKSIZE);
            stack.add(root);
            do {
                final Node node = stack.remove(stack.size() - 1);
                if (DbvtAabbMm.Intersect(node.volume, origin, invdir, signs)) {
                    if (node.isinternal()) {
                        stack.add(node.childs[0]);
                        stack.add(node.childs[1]);
                    } else {
                        policy.Process(node);
                    }
                }
            }
            while (stack.size() != 0);
        }
    }

    public static void collideKDOP(final Node root, final Vector3f[] normals, final float[] offsets, final int count,
        final ICollide policy) {
        //DBVT_CHECKTYPE
        if (root != null) {
            final int inside = (1 << count) - 1;
            final ObjectArrayList<sStkNP> stack = new ObjectArrayList<>(SIMPLE_STACKSIZE);
            final int[] signs = new int[4 * 8];
            assert (count < (/*sizeof(signs)*/128 / /*sizeof(signs[0])*/ 4));
            for (int i = 0; i < count; ++i) {
                signs[i] = ((normals[i].x >= 0) ? 1 : 0) +
                    ((normals[i].y >= 0) ? 2 : 0) +
                    ((normals[i].z >= 0) ? 4 : 0);
            }
            stack.add(new sStkNP(root, 0));
            do {
                final sStkNP se = stack.remove(stack.size() - 1);
                boolean out = false;
                for (int i = 0, j = 1; (!out) && (i < count); ++i, j <<= 1) {
                    if (0 == (se.mask & j)) {
                        final int side = se.node.volume.Classify(normals[i], offsets[i], signs[i]);
                        switch (side) {
                            case -1:
                                out = true;
                                break;
                            case +1:
                                se.mask |= j;
                                break;
                        }
                    }
                }
                if (!out) {
                    if ((se.mask != inside) && (se.node.isinternal())) {
                        stack.add(new sStkNP(se.node.childs[0], se.mask));
                        stack.add(new sStkNP(se.node.childs[1], se.mask));
                    } else {
                        if (policy.AllLeaves(se.node)) {
                            enumLeaves(se.node, policy);
                        }
                    }
                }
            }
            while (stack.size() != 0);
        }
    }

    public static void collideOCL(final Node root, final Vector3f[] normals, final float[] offsets,
        final Vector3f sortaxis, final int count,
        final ICollide policy) {
        collideOCL(root, normals, offsets, sortaxis, count, policy, true);
    }

    public static void collideOCL(final Node root, final Vector3f[] normals, final float[] offsets,
        final Vector3f sortaxis, final int count,
        final ICollide policy, final boolean fullsort) {
        //DBVT_CHECKTYPE
        if (root != null) {
            final int srtsgns = (sortaxis.x >= 0 ? 1 : 0) +
                (sortaxis.y >= 0 ? 2 : 0) +
                (sortaxis.z >= 0 ? 4 : 0);
            final int inside = (1 << count) - 1;
            final ObjectArrayList<sStkNPS> stock = new ObjectArrayList<>();
            final IntArrayList ifree = new IntArrayList();
            final IntArrayList stack = new IntArrayList();
            final int[] signs = new int[/*sizeof(unsigned)*8*/4 * 8];
            assert (count < (/*sizeof(signs)*/128 / /*sizeof(signs[0])*/ 4));
            for (int i = 0; i < count; i++) {
                signs[i] = ((normals[i].x >= 0) ? 1 : 0) +
                    ((normals[i].y >= 0) ? 2 : 0) +
                    ((normals[i].z >= 0) ? 4 : 0);
            }
            //stock.reserve(SIMPLE_STACKSIZE);
            //stack.reserve(SIMPLE_STACKSIZE);
            //ifree.reserve(SIMPLE_STACKSIZE);
            stack.add(allocate(ifree, stock, new sStkNPS(root, 0, root.volume.ProjectMinimum(sortaxis, srtsgns))));
            do {
                // JAVA NOTE: check
                final int id = stack.remove(stack.size() - 1);
                final sStkNPS se = stock.getQuick(id);
                ifree.add(id);
                if (se.mask != inside) {
                    boolean out = false;
                    for (int i = 0, j = 1; (!out) && (i < count); ++i, j <<= 1) {
                        if (0 == (se.mask & j)) {
                            final int side = se.node.volume.Classify(normals[i], offsets[i], signs[i]);
                            switch (side) {
                                case -1:
                                    out = true;
                                    break;
                                case +1:
                                    se.mask |= j;
                                    break;
                            }
                        }
                    }
                    if (out) {
                        continue;
                    }
                }
                if (policy.Descent(se.node)) {
                    if (se.node.isinternal()) {
                        final Node[] pns = new Node[] {se.node.childs[0], se.node.childs[1]};
                        final sStkNPS[] nes = new sStkNPS[] {
                            new sStkNPS(pns[0], se.mask, pns[0].volume.ProjectMinimum(sortaxis, srtsgns)),
                            new sStkNPS(pns[1], se.mask, pns[1].volume.ProjectMinimum(sortaxis, srtsgns))
                        };
                        final int q = nes[0].value < nes[1].value ? 1 : 0;
                        int j = stack.size();
                        if (fullsort && (j > 0)) {
                            /* Insert 0	*/
                            j = nearest(stack, stock, nes[q].value, 0, stack.size());
                            stack.add(0);
                            //#if DBVT_USE_MEMMOVE
                            //memmove(&stack[j+1],&stack[j],sizeof(int)*(stack.size()-j-1));
                            //#else
                            for (int k = stack.size() - 1; k > j; --k) {
                                stack.set(k, stack.get(k - 1));
                                //#endif
                            }
                            stack.set(j, allocate(ifree, stock, nes[q]));
                            /* Insert 1	*/
                            j = nearest(stack, stock, nes[1 - q].value, j, stack.size());
                            stack.add(0);
                            //#if DBVT_USE_MEMMOVE
                            //memmove(&stack[j+1],&stack[j],sizeof(int)*(stack.size()-j-1));
                            //#else
                            for (int k = stack.size() - 1; k > j; --k) {
                                stack.set(k, stack.get(k - 1));
                                //#endif
                            }
                            stack.set(j, allocate(ifree, stock, nes[1 - q]));
                        } else {
                            stack.add(allocate(ifree, stock, nes[q]));
                            stack.add(allocate(ifree, stock, nes[1 - q]));
                        }
                    } else {
                        policy.Process(se.node, se.value);
                    }
                }
            }
            while (stack.size() != 0);
        }
    }

    public static void collideTU(final Node root, final ICollide policy) {
        //DBVT_CHECKTYPE
        if (root != null) {
            final ObjectArrayList<Node> stack = new ObjectArrayList<>(SIMPLE_STACKSIZE);
            stack.add(root);
            do {
                final Node n = stack.remove(stack.size() - 1);
                if (policy.Descent(n)) {
                    if (n.isinternal()) {
                        stack.add(n.childs[0]);
                        stack.add(n.childs[1]);
                    } else {
                        policy.Process(n);
                    }
                }
            }
            while (stack.size() > 0);
        }
    }

    public static int nearest(final IntArrayList i, final ObjectArrayList<sStkNPS> a, final float v, int l, int h) {
        int m = 0;
        while (l < h) {
            m = (l + h) >> 1;
            if (a.getQuick(i.get(m)).value >= v) {
                l = m + 1;
            } else {
                h = m;
            }
        }
        return h;
    }

    public static int allocate(final IntArrayList ifree, final ObjectArrayList<sStkNPS> stock, final sStkNPS value) {
        final int i;
        if (ifree.size() > 0) {
            i = ifree.get(ifree.size() - 1);
            ifree.remove(ifree.size() - 1);
            stock.getQuick(i).set(value);
        } else {
            i = stock.size();
            stock.add(value);
        }
        return (i);
    }

    ////////////////////////////////////////////////////////////////////////////

    private static int indexof(final Node node) {
        return (node.parent.childs[1] == node) ? 1 : 0;
    }

    private static DbvtAabbMm merge(final DbvtAabbMm a, final DbvtAabbMm b, final DbvtAabbMm out) {
        DbvtAabbMm.Merge(a, b, out);
        return out;
    }

    // volume+edge lengths
    private static float size(final DbvtAabbMm a) {
        final Vector3f edges = a.Lengths(new Vector3f()); // Stack.alloc(Vector3f.class));
        return (edges.x * edges.y * edges.z +
            edges.x + edges.y + edges.z);
    }

    private static void deletenode(final Dbvt pdbvt, final Node node) {
        //btAlignedFree(pdbvt->m_free);
        pdbvt.free = node;
    }

    private static void recursedeletenode(final Dbvt pdbvt, final Node node) {
        if (!node.isleaf()) {
            recursedeletenode(pdbvt, node.childs[0]);
            recursedeletenode(pdbvt, node.childs[1]);
        }
        if (node == pdbvt.root) {
            pdbvt.root = null;
        }
        deletenode(pdbvt, node);
    }

    private static Node createnode(final Dbvt pdbvt, final Node parent, final DbvtAabbMm volume, final Object data) {
        final Node node;
        if (pdbvt.free != null) {
            node = pdbvt.free;
            pdbvt.free = null;
        } else {
            node = new Node();
        }
        node.parent = parent;
        node.volume.set(volume);
        node.data = data;
        node.childs[1] = null;
        return node;
    }

    private static void insertleaf(final Dbvt pdbvt, Node root, final Node leaf) {
        if (pdbvt.root == null) {
            pdbvt.root = leaf;
            leaf.parent = null;
        } else {
            if (!root.isleaf()) {
                do {
                    if (DbvtAabbMm.Proximity(root.childs[0].volume, leaf.volume) <
                        DbvtAabbMm.Proximity(root.childs[1].volume, leaf.volume)) {
                        root = root.childs[0];
                    } else {
                        root = root.childs[1];
                    }
                }
                while (!root.isleaf());
            }
            Node prev = root.parent;
            Node node = createnode(pdbvt, prev, merge(leaf.volume, root.volume, new DbvtAabbMm()), null);
            if (prev != null) {
                prev.childs[indexof(root)] = node;
                node.childs[0] = root;
                root.parent = node;
                node.childs[1] = leaf;
                leaf.parent = node;
                do {
                    if (!prev.volume.Contain(node.volume)) {
                        DbvtAabbMm.Merge(prev.childs[0].volume, prev.childs[1].volume, prev.volume);
                    } else {
                        break;
                    }
                    node = prev;
                }
                while (null != (prev = node.parent));
            } else {
                node.childs[0] = root;
                root.parent = node;
                node.childs[1] = leaf;
                leaf.parent = node;
                pdbvt.root = node;
            }
        }
    }

    private static Node removeleaf(final Dbvt pdbvt, final Node leaf) {
        if (leaf == pdbvt.root) {
            pdbvt.root = null;
            return null;
        } else {
            final Node parent = leaf.parent;
            Node prev = parent.parent;
            final Node sibling = parent.childs[1 - indexof(leaf)];
            if (prev != null) {
                prev.childs[indexof(parent)] = sibling;
                sibling.parent = prev;
                deletenode(pdbvt, parent);
                while (prev != null) {
                    final DbvtAabbMm pb = prev.volume;
                    DbvtAabbMm.Merge(prev.childs[0].volume, prev.childs[1].volume, prev.volume);
                    if (DbvtAabbMm.NotEqual(pb, prev.volume)) {
                        prev = prev.parent;
                    } else {
                        break;
                    }
                }
                return (prev != null ? prev : pdbvt.root);
            } else {
                pdbvt.root = sibling;
                sibling.parent = null;
                deletenode(pdbvt, parent);
                return pdbvt.root;
            }
        }
    }

    private static void fetchleaves(final Dbvt pdbvt, final Node root, final ObjectArrayList<Node> leaves) {
        fetchleaves(pdbvt, root, leaves, -1);
    }

    private static void fetchleaves(final Dbvt pdbvt, final Node root, final ObjectArrayList<Node> leaves,
        final int depth) {
        if (root.isinternal() && depth != 0) {
            fetchleaves(pdbvt, root.childs[0], leaves, depth - 1);
            fetchleaves(pdbvt, root.childs[1], leaves, depth - 1);
            deletenode(pdbvt, root);
        } else {
            leaves.add(root);
        }
    }

    private static void split(
        final ObjectArrayList<Node> leaves, final ObjectArrayList<Node> left, final ObjectArrayList<Node> right,
        final Vector3f org, final Vector3f axis) {
        final Vector3f tmp = new Vector3f(); // Stack.alloc(Vector3f.class);
        MiscUtil.resize(left, 0, Node.class);
        MiscUtil.resize(right, 0, Node.class);
        for (int i = 0, ni = leaves.size(); i < ni; i++) {
            leaves.getQuick(i).volume.Center(tmp);
            tmp.sub(org);
            if (axis.dot(tmp) < 0f) {
                left.add(leaves.getQuick(i));
            } else {
                right.add(leaves.getQuick(i));
            }
        }
    }

    private static DbvtAabbMm bounds(final ObjectArrayList<Node> leaves) {
        final DbvtAabbMm volume = new DbvtAabbMm(leaves.getQuick(0).volume);
        for (int i = 1, ni = leaves.size(); i < ni; i++) {
            merge(volume, leaves.getQuick(i).volume, volume);
        }
        return volume;
    }

    private static void bottomup(final Dbvt pdbvt, final ObjectArrayList<Node> leaves) {
        final DbvtAabbMm tmpVolume = new DbvtAabbMm();
        while (leaves.size() > 1) {
            float minsize = BulletGlobals.SIMD_INFINITY;
            final int[] minidx = new int[] {-1, -1};
            for (int i = 0; i < leaves.size(); i++) {
                for (int j = i + 1; j < leaves.size(); j++) {
                    final float sz = size(merge(leaves.getQuick(i).volume, leaves.getQuick(j).volume, tmpVolume));
                    if (sz < minsize) {
                        minsize = sz;
                        minidx[0] = i;
                        minidx[1] = j;
                    }
                }
            }
            final Node[] n = new Node[] {leaves.getQuick(minidx[0]), leaves.getQuick(minidx[1])};
            final Node p = createnode(pdbvt, null, merge(n[0].volume, n[1].volume, new DbvtAabbMm()), null);
            p.childs[0] = n[0];
            p.childs[1] = n[1];
            n[0].parent = p;
            n[1].parent = p;
            // JAVA NOTE: check
            leaves.setQuick(minidx[0], p);
            Collections.swap(leaves, minidx[1], leaves.size() - 1);
            leaves.removeQuick(leaves.size() - 1);
        }
    }

    private static Vector3f[] axis =
        new Vector3f[] {new Vector3f(1, 0, 0), new Vector3f(0, 1, 0), new Vector3f(0, 0, 1)};

    private static Node topdown(final Dbvt pdbvt, final ObjectArrayList<Node> leaves, final int bu_treshold) {
        if (leaves.size() > 1) {
            if (leaves.size() > bu_treshold) {
                final DbvtAabbMm vol = bounds(leaves);
                final Vector3f org = vol.Center(new Vector3f()); // Stack.alloc(Vector3f.class));
                final ObjectArrayList[] sets = new ObjectArrayList[2];
                for (int i = 0; i < sets.length; i++) {
                    sets[i] = new ObjectArrayList();
                }
                int bestaxis = -1;
                int bestmidp = leaves.size();
                final int[][] splitcount = new int[/*3*/][/*2*/] {{0, 0}, {0, 0}, {0, 0}};

                final Vector3f x = new Vector3f(); // Stack.alloc(Vector3f.class);

                for (int i = 0; i < leaves.size(); i++) {
                    leaves.getQuick(i).volume.Center(x);
                    x.sub(org);
                    for (int j = 0; j < 3; j++) {
                        splitcount[j][x.dot(axis[j]) > 0f ? 1 : 0]++;
                    }
                }
                for (int i = 0; i < 3; i++) {
                    if ((splitcount[i][0] > 0) && (splitcount[i][1] > 0)) {
                        final int midp = Math.abs(splitcount[i][0] - splitcount[i][1]);
                        if (midp < bestmidp) {
                            bestaxis = i;
                            bestmidp = midp;
                        }
                    }
                }
                if (bestaxis >= 0) {
                    //sets[0].reserve(splitcount[bestaxis][0]);
                    //sets[1].reserve(splitcount[bestaxis][1]);
                    split(leaves, sets[0], sets[1], org, axis[bestaxis]);
                } else {
                    //sets[0].reserve(leaves.size()/2+1);
                    //sets[1].reserve(leaves.size()/2);
                    for (int i = 0, ni = leaves.size(); i < ni; i++) {
                        sets[i & 1].add(leaves.getQuick(i));
                    }
                }
                final Node node = createnode(pdbvt, null, vol, null);
                node.childs[0] = topdown(pdbvt, sets[0], bu_treshold);
                node.childs[1] = topdown(pdbvt, sets[1], bu_treshold);
                node.childs[0].parent = node;
                node.childs[1].parent = node;
                return node;
            } else {
                bottomup(pdbvt, leaves);
                return leaves.getQuick(0);
            }
        }
        return leaves.getQuick(0);
    }

    private static Node sort(final Node n, final Node[] r) {
        final Node p = n.parent;
        assert (n.isinternal());
        // JAVA TODO: fix this
        if (p != null && p.hashCode() > n.hashCode()) {
            final int i = indexof(n);
            final int j = 1 - i;
            final Node s = p.childs[j];
            final Node q = p.parent;
            assert (n == p.childs[i]);
            if (q != null) {
                q.childs[indexof(p)] = n;
            } else {
                r[0] = n;
            }
            s.parent = n;
            p.parent = n;
            n.parent = q;
            p.childs[0] = n.childs[0];
            p.childs[1] = n.childs[1];
            n.childs[0].parent = p;
            n.childs[1].parent = p;
            n.childs[i] = p;
            n.childs[j] = s;

            DbvtAabbMm.swap(p.volume, n.volume);
            return p;
        }
        return n;
    }

    private static Node walkup(Node n, int count) {
        while (n != null && (count--) != 0) {
            n = n.parent;
        }
        return n;
    }

    ////////////////////////////////////////////////////////////////////////////

    public static class Node {
        public final DbvtAabbMm volume = new DbvtAabbMm();
        public Node parent;
        public final Node[] childs = new Node[2];
        public Object data;

        public boolean isleaf() {
            return childs[1] == null;
        }

        public boolean isinternal() {
            return !isleaf();
        }
    }

    /**
     * Stack element
     */
    public static class sStkNN {
        public Node a;
        public Node b;

        public sStkNN(final Node na, final Node nb) {
            a = na;
            b = nb;
        }
    }

    public static class sStkNP {
        public Node node;
        public int mask;

        public sStkNP(final Node n, final int m) {
            node = n;
            mask = m;
        }
    }

    public static class sStkNPS {
        public Node node;
        public int mask;
        public float value;

        public sStkNPS() {
        }

        public sStkNPS(final Node n, final int m, final float v) {
            node = n;
            mask = m;
            value = v;
        }

        public void set(final sStkNPS o) {
            node = o.node;
            mask = o.mask;
            value = o.value;
        }
    }

    public static class sStkCLN {
        public Node node;
        public Node parent;

        public sStkCLN(final Node n, final Node p) {
            node = n;
            parent = p;
        }
    }

    public static class ICollide {
        public void Process(final Node n1, final Node n2) {
        }

        public void Process(final Node n) {
        }

        public void Process(final Node n, final float f) {
            Process(n);
        }

        public boolean Descent(final Node n) {
            return true;
        }

        public boolean AllLeaves(final Node n) {
            return true;
        }
    }

    public static abstract class IWriter {
        public abstract void Prepare(Node root, int numnodes);

        public abstract void WriteNode(Node n, int index, int parent, int child0, int child1);

        public abstract void WriteLeaf(Node n, int index, int parent);
    }

    public static class IClone {
        public void CloneLeaf(final Node n) {
        }
    }

}
