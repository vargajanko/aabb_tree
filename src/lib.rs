use arrayvec::ArrayVec;
use bevy_ecs::system::Resource;
use bevy_math::Vec3;

#[derive(Resource)]
pub struct Tree {
    pub nodes: Vec<Node>,
    pub root_index: usize,
    pub free_list_entry: usize,
}

#[derive(Resource)]
pub struct MovedAabbs {
    pub moved_aabbs: Vec<usize>,
}

impl Default for MovedAabbs {
    fn default() -> Self {
        MovedAabbs {
            moved_aabbs: Vec::new(),
        }
    }
}

impl Default for Tree {
    fn default() -> Self {
        Self::new()
    }
}

impl Tree {
    pub fn new() -> Tree {
        let v: Vec<Node> = Vec::with_capacity(16);
        let mut tree = Tree {
            nodes: v,
            root_index: usize::MAX,
            free_list_entry: 0,
        };
        tree.nodes.resize_with(16, Node::default);
        let size = tree.nodes.len() - 1;
        // Build a linked list for the free list.
        for i in 0..size {
            tree.nodes[i].parent_or_free.next = i + 1;
            tree.nodes[i].height = -1;
        }
        tree.nodes[size].parent_or_free.next = usize::MAX;
        tree.nodes[size].height = -1;
        tree
    }
    ///unsafe for accesing the union
    fn allocate_leaf_node(&mut self) -> usize {
        if self.free_list_entry == usize::MAX {
            let size = self.nodes.len();
            self.nodes.resize_with(self.nodes.len() * 2, Node::default);
            let new_size = self.nodes.len() - 1;
            for i in size..new_size {
                self.nodes[i].parent_or_free.next = i + 1;
                self.nodes[i].height = -1;
            }

            self.nodes[new_size].parent_or_free.next = usize::MAX;
            self.nodes[new_size].height = -1;
            self.free_list_entry = size;
        }

        let node_id = self.free_list_entry;
        unsafe {
            self.free_list_entry = self.nodes[node_id].parent_or_free.next;
        }
        self.nodes[node_id].parent_or_free.parent = usize::MAX;
        self.nodes[node_id].child1 = usize::MAX;
        self.nodes[node_id].child2 = usize::MAX;
        self.nodes[node_id].height = 0;

        node_id
    }

    ///Push to be freed node in front of freelist.
    fn free_node(&mut self, node_id: usize) {
        self.nodes[node_id].parent_or_free.next = self.free_list_entry;
        self.nodes[node_id].height = -1;
        self.free_list_entry = node_id;
    }

    pub fn create_aabb(&mut self, aabb: &Aabb) -> usize {
        let proxy_id = self.allocate_leaf_node();

        self.nodes[proxy_id].aabb_box.lower_bound = aabb.lower_bound;
        self.nodes[proxy_id].aabb_box.upper_bound = aabb.upper_bound;
        //self.nodes[proxyId].userData = userData;
        self.nodes[proxy_id].height = 0;
        self.nodes[proxy_id].moved = false;
        //self.nodes[proxyId].moved = true;

        self.insert_leaf(proxy_id);

        proxy_id
    }

    pub fn destroy_aabb(&mut self, aabb_id: usize) {
        self.remove_parent(aabb_id);
        self.free_node(aabb_id);
    }

    pub fn move_aabb(&mut self, aabb_id: usize, aabb: &Aabb) {
        self.remove_parent(aabb_id);
        self.nodes[aabb_id].aabb_box.lower_bound = aabb.lower_bound;
        self.nodes[aabb_id].aabb_box.upper_bound = aabb.upper_bound;
        //self.nodes[proxyId].userData = userData;
        self.nodes[aabb_id].height = 0;
        self.nodes[aabb_id].moved = true;
        self.insert_leaf(aabb_id);
    }

    fn insert_leaf(&mut self, leaf: usize) {
        if self.root_index == usize::MAX {
            self.root_index = leaf;
            self.nodes[leaf].parent_or_free.parent = usize::MAX;
            return;
        }
        // Find the best sibling for this node
        let leaf_aabb = self.nodes[leaf].aabb_box;
        let mut index = self.root_index;
        while !self.nodes[index].is_leaf() {
            let child1 = self.nodes[index].child1;
            let child2 = self.nodes[index].child2;

            let area = self.nodes[index].aabb_box.surface_area();
            let combined_aabb = union(&self.nodes[index].aabb_box, &leaf_aabb);
            let combined_area = combined_aabb.surface_area();

            // Cost of creating a new parent for this node and the new leaf
            let cost = 2.0 * combined_area;

            // Minimum cost of pushing the leaf further down the tree
            let inheritance_cost = 2.0 * (combined_area - area);

            let cost1 = if self.nodes[child1].is_leaf() {
                let aabb = union(&leaf_aabb, &self.nodes[child1].aabb_box);
                aabb.surface_area() + inheritance_cost
            } else {
                let aabb = union(&leaf_aabb, &self.nodes[child1].aabb_box);
                let old_area = self.nodes[child1].aabb_box.surface_area();
                let new_area = aabb.surface_area();
                (new_area - old_area) + inheritance_cost
            };

            let cost2 = if self.nodes[child2].is_leaf() {
                let aabb = union(&leaf_aabb, &self.nodes[child2].aabb_box);
                aabb.surface_area() + inheritance_cost
            } else {
                let aabb = union(&leaf_aabb, &self.nodes[child2].aabb_box);
                let old_area = self.nodes[child2].aabb_box.surface_area();
                let new_area = aabb.surface_area();
                (new_area - old_area) + inheritance_cost
            };

            // Descend according to the minimum cost.
            if cost < cost1 && cost < cost2 {
                break;
            }

            // Descend
            if cost1 < cost2 {
                index = child1;
            } else {
                index = child2;
            }
        }

        let sibling = index;

        //Create a new parent.
        let old_parent;
        unsafe {
            old_parent = self.nodes[sibling].parent_or_free.parent;
        }
        let new_parent = self.allocate_leaf_node();
        self.nodes[new_parent].parent_or_free.parent = old_parent;
        self.nodes[new_parent].aabb_box = union(&leaf_aabb, &self.nodes[sibling].aabb_box);
        self.nodes[new_parent].height = self.nodes[sibling].height + 1;
        //self.nodes[new_parent].aabb_userdata

        if old_parent != usize::MAX {
            if self.nodes[old_parent].child1 == sibling {
                self.nodes[old_parent].child1 = new_parent;
            } else {
                self.nodes[old_parent].child2 = new_parent;
            }

            self.nodes[new_parent].child1 = sibling;
            self.nodes[new_parent].child2 = leaf;
            self.nodes[sibling].parent_or_free.parent = new_parent;
            self.nodes[leaf].parent_or_free.parent = new_parent;
        } else {
            // The sibling was the root.
            self.nodes[new_parent].child1 = sibling;
            self.nodes[new_parent].child2 = leaf;
            self.nodes[sibling].parent_or_free.parent = new_parent;
            self.nodes[leaf].parent_or_free.parent = new_parent;
            self.root_index = new_parent;
        }

        // Walk back up the tree fixing heights and AABBs
        unsafe {
            index = self.nodes[leaf].parent_or_free.parent;
        }
        while index != usize::MAX {
            index = self.balance(index);

            let child1 = self.nodes[index].child1;
            let child2 = self.nodes[index].child2;

            self.nodes[index].height = 1 + self.nodes[child1].height.max(self.nodes[child2].height);
            self.nodes[index].aabb_box =
                union(&self.nodes[child1].aabb_box, &self.nodes[child2].aabb_box);
            unsafe {
                index = self.nodes[index].parent_or_free.parent;
            }
        }
    }

    pub fn query_all_leafs(&self, check_id: usize) {
        let mut stack = ArrayVec::<usize, 256>::new();
        stack.push(self.root_index);
        while !stack.is_empty() {
            let node_id = stack.pop().unwrap();
            if node_id == usize::MAX {
                continue;
            }

            let node = &self.nodes[node_id];

            if node.aabb_box.overlap(&self.nodes[check_id].aabb_box) {
                if node.is_leaf() {
                    if !(node.moved && check_id >= node_id) {
                        println!("zep aabb {} {} are overlaping", node_id, check_id);
                    }
                } else {
                    stack.push(node.child1);
                    stack.push(node.child2);
                }
            }
        }
    }

    fn balance(&mut self, i_a: usize) -> usize {
        unsafe {
            //let a = &self.nodes[i_a];
            if self.nodes[i_a].is_leaf() || self.nodes[i_a].height < 2 {
                return i_a;
            }

            let i_b = self.nodes[i_a].child1;
            let i_c = self.nodes[i_a].child2;

            //let b = &mut self.nodes[i_b];
            //let c = &mut self.nodes[i_c];

            let balance_height = self.nodes[i_c].height - self.nodes[i_b].height;

            // rotate C up
            if balance_height > 1 {
                let i_f = self.nodes[i_c].child1;
                let i_g = self.nodes[i_c].child2;

                // let f = &mut self.nodes[i_f];
                // let g = &mut self.nodes[i_g];

                //swap a and c
                self.nodes[i_c].child1 = i_a;
                self.nodes[i_c].parent_or_free.parent = self.nodes[i_a].parent_or_free.parent;
                self.nodes[i_a].parent_or_free.parent = i_c;

                // a old parent should point to c
                if self.nodes[i_c].parent_or_free.parent != usize::MAX {
                    if self.nodes[self.nodes[i_c].parent_or_free.parent].child1 == i_a {
                        let i = self.nodes[i_c].parent_or_free.parent;
                        self.nodes[i].child1 = i_c;
                    } else {
                        let i = self.nodes[i_c].parent_or_free.parent;
                        self.nodes[i].child2 = i_c;
                    }
                } else {
                    self.root_index = i_c;
                }

                // Rotate
                if self.nodes[i_f].height > self.nodes[i_g].height {
                    self.nodes[i_c].child2 = i_f;
                    self.nodes[i_a].child2 = i_g;
                    self.nodes[i_g].parent_or_free.parent = i_a;
                    self.nodes[i_a].aabb_box =
                        union(&self.nodes[i_b].aabb_box, &self.nodes[i_g].aabb_box);
                    self.nodes[i_c].aabb_box =
                        union(&self.nodes[i_a].aabb_box, &self.nodes[i_f].aabb_box);

                    self.nodes[i_a].height = 1 + self.nodes[i_b].height.max(self.nodes[i_g].height);
                    self.nodes[i_c].height = 1 + self.nodes[i_a].height.max(self.nodes[i_f].height);
                } else {
                    self.nodes[i_c].child2 = i_g;
                    self.nodes[i_a].child2 = i_f;
                    self.nodes[i_f].parent_or_free.parent = i_a;
                    self.nodes[i_a].aabb_box =
                        union(&self.nodes[i_b].aabb_box, &self.nodes[i_f].aabb_box);
                    self.nodes[i_c].aabb_box =
                        union(&self.nodes[i_a].aabb_box, &self.nodes[i_g].aabb_box);

                    self.nodes[i_a].height = 1 + self.nodes[i_b].height.max(self.nodes[i_g].height);
                    self.nodes[i_c].height = 1 + self.nodes[i_a].height.max(self.nodes[i_f].height);
                }

                return i_c;
            }

            // rotate b up
            if balance_height < -1 {
                let i_d = self.nodes[i_b].child1;
                let i_e = self.nodes[i_b].child2;

                // let f = &mut self.nodes[i_f];
                // let g = &mut self.nodes[i_g];

                //swap a and c
                self.nodes[i_b].child1 = i_a;
                self.nodes[i_b].parent_or_free.parent = self.nodes[i_a].parent_or_free.parent;
                self.nodes[i_a].parent_or_free.parent = i_b;

                // a old parent should point to c
                if self.nodes[i_b].parent_or_free.parent != usize::MAX {
                    if self.nodes[self.nodes[i_b].parent_or_free.parent].child1 == i_a {
                        let i = self.nodes[i_b].parent_or_free.parent;
                        self.nodes[i].child1 = i_b;
                    } else {
                        let i = self.nodes[i_b].parent_or_free.parent;
                        self.nodes[i].child2 = i_b;
                    }
                } else {
                    self.root_index = i_b;
                }

                // Rotate
                if self.nodes[i_d].height > self.nodes[i_e].height {
                    self.nodes[i_b].child2 = i_d;
                    self.nodes[i_a].child2 = i_e;
                    self.nodes[i_e].parent_or_free.parent = i_a;

                    self.nodes[i_a].aabb_box =
                        union(&self.nodes[i_c].aabb_box, &self.nodes[i_e].aabb_box);
                    self.nodes[i_b].aabb_box =
                        union(&self.nodes[i_a].aabb_box, &self.nodes[i_d].aabb_box);

                    self.nodes[i_a].height = 1 + self.nodes[i_c].height.max(self.nodes[i_e].height);
                    self.nodes[i_b].height = 1 + self.nodes[i_a].height.max(self.nodes[i_d].height);
                } else {
                    self.nodes[i_b].child2 = i_e;
                    self.nodes[i_a].child2 = i_d;
                    self.nodes[i_d].parent_or_free.parent = i_a;

                    self.nodes[i_a].aabb_box =
                        union(&self.nodes[i_c].aabb_box, &self.nodes[i_d].aabb_box);
                    self.nodes[i_b].aabb_box =
                        union(&self.nodes[i_a].aabb_box, &self.nodes[i_e].aabb_box);

                    self.nodes[i_a].height = 1 + self.nodes[i_c].height.max(self.nodes[i_d].height);
                    self.nodes[i_b].height = 1 + self.nodes[i_a].height.max(self.nodes[i_e].height);
                }

                return i_b;
            }

            i_a
        }
    }

    fn remove_parent(&mut self, leaf: usize) {
        if leaf == self.root_index {
            self.root_index = usize::MAX;
        }

        let parent;
        let grand_parent;
        unsafe {
            parent = self.nodes[leaf].parent_or_free.parent;
            grand_parent = self.nodes[parent].parent_or_free.parent;
        }

        let sibling = if self.nodes[parent].child1 == leaf {
            self.nodes[parent].child2
        } else {
            self.nodes[parent].child1
        };

        if grand_parent != usize::MAX {
            if self.nodes[grand_parent].child1 == parent {
                self.nodes[grand_parent].child1 = sibling;
            } else {
                self.nodes[grand_parent].child2 = sibling;
            }
            self.nodes[sibling].parent_or_free.parent = grand_parent;
            self.free_node(parent);

            //adjust anector bounds
            let mut index = grand_parent;
            while index != usize::MAX {
                index = self.balance(index);

                let child1 = self.nodes[index].child1;
                let child2 = self.nodes[index].child2;

                self.nodes[index].aabb_box =
                    union(&self.nodes[child1].aabb_box, &self.nodes[child2].aabb_box);
                self.nodes[index].height =
                    1 + self.nodes[child2].height.max(self.nodes[child1].height);
            }
        } else {
            self.root_index = sibling;
            self.nodes[sibling].parent_or_free.parent = usize::MAX;
            self.free_node(parent);
        }
    }
}

pub union NodeType {
    parent: usize,
    next: usize,
}

impl Default for NodeType {
    fn default() -> Self {
        NodeType { parent: 0 }
    }
}

#[derive(Default)]
pub struct Node {
    pub aabb_box: Aabb,
    pub object_index: usize,
    pub moved: bool,
    pub parent_or_free: NodeType,
    pub child1: usize,
    pub child2: usize,
    // leaf = 0, free node = -1
    pub height: i16,
}

impl Node {
    pub fn is_leaf(&self) -> bool {
        self.height == 0
    }
}

#[derive(Default, Clone, Copy, Debug)]
pub struct Aabb {
    pub lower_bound: Vec3,
    pub upper_bound: Vec3,
}

impl Aabb {
    pub fn min(&self, rhs: &Aabb) -> Vec3 {
        Vec3::new(
            if self.lower_bound.x < rhs.lower_bound.x {
                self.lower_bound.x
            } else {
                rhs.lower_bound.x
            },
            if self.lower_bound.y < rhs.lower_bound.y {
                self.lower_bound.y
            } else {
                rhs.lower_bound.y
            },
            if self.lower_bound.z < rhs.lower_bound.z {
                self.lower_bound.z
            } else {
                rhs.lower_bound.z
            },
        )
    }

    pub fn max(&self, rhs: &Aabb) -> Vec3 {
        Vec3 {
            x: if self.lower_bound.x > rhs.lower_bound.x {
                self.lower_bound.x
            } else {
                rhs.lower_bound.x
            },
            y: if self.lower_bound.y > rhs.lower_bound.y {
                self.lower_bound.y
            } else {
                rhs.lower_bound.y
            },
            z: if self.lower_bound.z > rhs.lower_bound.z {
                self.lower_bound.z
            } else {
                rhs.lower_bound.z
            },
        }
    }

    pub fn new(lower: Vec3, upper: Vec3) -> Aabb {
        Aabb {
            lower_bound: lower,
            upper_bound: upper,
        }
    }

    pub fn overlap(&self, rhs: &Aabb) -> bool {
        let d1 = rhs.lower_bound - self.upper_bound;
        let d2 = self.lower_bound - rhs.upper_bound;

        if d1.x > 0.0 || d1.y > 0.0 || d1.z > 0.0 {
            return false;
        }

        if d2.x > 0.0 || d2.y > 0.0 || d2.z > 0.0 {
            return false;
        }

        true
    }

    //computes the surfacearea of a 3D AABB
    //SA(a) short for surface area of a
    pub fn surface_area(&self) -> f32 {
        let width = self.upper_bound - self.lower_bound;
        2.0 * (width.x * width.y + width.x * width.z + width.y * width.z)
    }
}

//add simd stuff. cool stuff to benchmark later.
// C = a U b;   U is the cup notation for the union of a and b
//(all points of b and a are in the union)
pub fn union(a: &Aabb, b: &Aabb) -> Aabb {
    Aabb::new(a.min(b), a.max(b))
}

//computes the surfacearea of a 3D AABB
//SA(a) short for surface area of a
pub fn surface_area(a: &Aabb) -> f32 {
    let width = a.upper_bound - a.lower_bound;
    2.0 * (width.x * width.y + width.x * width.z + width.y * width.z)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = 2 + 2;
        assert_eq!(result, 4);
    }
}
