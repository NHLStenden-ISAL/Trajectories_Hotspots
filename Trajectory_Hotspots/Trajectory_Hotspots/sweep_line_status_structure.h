#pragma once

namespace Segment_Intersection_Sweep_Line
{
    template<typename SegmentT>
    class Sweep_Line_Status_structure
    {
    public:
        Sweep_Line_Status_structure() : current_event_point(std::numeric_limits<float>::max(), std::numeric_limits<float>::max())
        {
        };

        explicit Sweep_Line_Status_structure(const Vec2& starting_event_point) : current_event_point(starting_event_point) {};

        class Node
        {
        public:
            explicit Node(const int segment) : segment(segment) {};

            int segment; //index
            int height = 0;
            std::unique_ptr<Node> left = nullptr;
            std::unique_ptr<Node> right = nullptr;
            const Node* parent = nullptr;

            void calculate_height();
            int height_difference();
            int get_right_neighbour(const std::vector<SegmentT>& segments, const Vec2& current_event_point) const;
            int get_left_neighbour(const std::vector<SegmentT>& segments, const Vec2& current_event_point) const;

            std::vector<int> get_all_neighbours(const std::vector<SegmentT>& segments, const Vec2& current_event_point,
                int& left_neighbour, int& right_neighbour) const;

            bool get_intersecting_left_to_right(const std::vector<SegmentT>& segments, const Vec2& current_event_point, std::vector<const Node*>& right_nodes) const;
            bool get_intersecting_right_to_left(const std::vector<SegmentT>& segments, const Vec2& current_event_point, std::vector<const Node*>& left_nodes) const;


            void print_tree(Node* root, int spacing, std::string& tree_string) const;
        };

        void insert(const std::vector<SegmentT>& segments, const int new_segment);
        void insert(const std::vector<SegmentT>& segments, const int new_segment, int& left_node, int& right_node);
        void remove(const std::vector<SegmentT>& segments, const int segment_to_remove);
        void remove(const std::vector<SegmentT>& segments, const int segment_to_remove, int& left_node, int& right_node);
        bool contains(const std::vector<SegmentT>& segments, const SegmentT* search_segment);

        void set_current_event_position(const Vec2& new_position) { current_event_point = new_position; };

        std::vector<int> get_all_nodes_on_point(
            const std::vector<SegmentT> segments,
            int& left_neighbour,
            int& right_neighbour) const;

        ////Returns the first segment that intersects on the given x-coordinate
        Node* get_node(const std::vector<SegmentT>& segments, const Float& x_position) const;

        std::string print_tree() const;

        bool empty() const { return root == nullptr; };

    private:
        //TODO: SegmentT* vector here, ref?

        std::unique_ptr<Node> root;

        Vec2 current_event_point;

        bool test_order_left_right(const std::vector<SegmentT>& segments, const int& new_segment, const Node& new_root) const;

        std::unique_ptr<Node> insert(std::unique_ptr<Node>&& node, const std::vector<SegmentT>& segments, const int new_segment, const Node*& added_node);
        std::unique_ptr<Node> add_to_subtree(std::unique_ptr<Node>&& root, const std::vector<SegmentT>& segments, const int new_segment, const Node*& added_node, const Node* parent);

        std::unique_ptr<Node> remove(std::unique_ptr<Node>&& node, const std::vector<SegmentT>& segments, const int segment_to_remove);
        std::unique_ptr<Node> remove_from_parent(std::unique_ptr<Node>&& parent, const std::vector<SegmentT>& segments, const int segment_to_remove);

        std::unique_ptr<Node> rotate_left(std::unique_ptr<Node>&& old_root);
        std::unique_ptr<Node> rotate_right(std::unique_ptr<Node>&& old_root);
        std::unique_ptr<Node> rotate_left_right(std::unique_ptr<Node>&& old_root);
        std::unique_ptr<Node> rotate_right_left(std::unique_ptr<Node>&& old_root);

        void print_tree(Node* root, int spacing, std::string& tree_string) const;
    };


    // Inserts a new node into the tree, rebalancing when needed.
    template<typename SegmentT>
    void Sweep_Line_Status_structure<SegmentT>::insert(const std::vector<SegmentT>& segments, const int new_segment)
    {
        if (root != nullptr)
        {
            const Node* node = nullptr;
            root = insert(std::move(root), segments, new_segment, node);
        }
        else
        {
            root = std::make_unique<Node>(new_segment);
        }
    }

    // Inserts a new node into the tree, rebalancing when needed. Also finds the left and right neighbour of the newly inserted node.
    template<typename SegmentT>
    void Sweep_Line_Status_structure<SegmentT>::insert(const std::vector<SegmentT>& segments, const int new_segment, int& left_node, int& right_node)
    {
        if (root != nullptr)
        {
            const Node* node = nullptr;
            root = insert(std::move(root), segments, new_segment, node);

            if (node != nullptr)
            {
                left_node = node->get_left_neighbour(segments, current_event_point);
                right_node = node->get_right_neighbour(segments, current_event_point);
            }
        }
        else
        {
            root = std::make_unique<Node>(new_segment);
        }
    }

    template<typename SegmentT>
    bool Sweep_Line_Status_structure<SegmentT>::test_order_left_right(const std::vector<SegmentT>& segments, const int& new_segment, const Node& new_root) const
    {
        Float current_x_position = segments.at(new_segment).y_intersect(current_event_point.y);
        Float node_x_position = segments.at(new_root.segment).y_intersect(current_event_point.y);


        if (current_x_position.is_inf())
        {
            //Horizontal segments come last in the set of segments that intersect the event point.
            //If these segments dont intersect the event point just use the ordering of the current event point and the other segment
            return current_event_point.x < node_x_position;
        }

        //A  = new_root-segment
        //B  = new_segment
        // CROSS A x B
        // if cross is negative, then the new segment is on the left

        Vec2 node_segment = *segments.at(new_root.segment).get_bottom_point() - Vec2(current_x_position, current_event_point.y);
        Vec2 new_segment_vec = *segments.at(new_segment).get_bottom_point() - Vec2(current_x_position, current_event_point.y);

        return (current_x_position < node_x_position || current_x_position == node_x_position && node_segment.cross(new_segment_vec) < 0.f);
    }

    template<typename SegmentT>
    std::unique_ptr<typename Sweep_Line_Status_structure<SegmentT>::Node> Sweep_Line_Status_structure<SegmentT>::insert(std::unique_ptr<Node>&& node, const std::vector<SegmentT>& segments, const int new_segment, const Node*& added_node)
    {
        std::unique_ptr<Node> new_root = std::move(node);

        if (test_order_left_right(segments, new_segment, *new_root.get()))
        {
            new_root->left = add_to_subtree(std::move(new_root->left), segments, new_segment, added_node, new_root.get());

            if (new_root->height_difference() == 2)
            {
                if (new_root->left->height_difference() >= 0)
                {
                    new_root = rotate_right(std::move(new_root));
                }
                else
                {
                    new_root = rotate_left_right(std::move(new_root));
                }
            }
        }
        else
        {
            new_root->right = add_to_subtree(std::move(new_root->right), segments, new_segment, added_node, new_root.get());

            if (new_root->height_difference() == -2)
            {
                if (new_root->right->height_difference() <= 0)
                {
                    new_root = rotate_left(std::move(new_root));
                }
                else
                {
                    new_root = rotate_right_left(std::move(new_root));
                }
            }
        }
        new_root->calculate_height();

        return new_root;
    }

    template<typename SegmentT>
    std::unique_ptr<typename Sweep_Line_Status_structure<SegmentT>::Node> Sweep_Line_Status_structure<SegmentT>::add_to_subtree(std::unique_ptr<Node>&& root, const std::vector<SegmentT>& segments, const int new_segment, const Node*& added_node, const Node* parent)
    {
        if (root == nullptr)
        {
            std::unique_ptr<Node> new_leaf_node = std::make_unique<Node>(new_segment);
            new_leaf_node->parent = parent;

            added_node = new_leaf_node.get();
            return new_leaf_node;
        }
        else
        {
            return insert(std::move(root), segments, new_segment, added_node);
        }
    }

    // Inserts a node from the tree, rebalancing when needed.
    template<typename SegmentT>
    void Sweep_Line_Status_structure<SegmentT>::remove(const std::vector<SegmentT>& segments, const int segment_to_remove)
    {
        if (root != nullptr)
        {
            root = remove(std::move(root), segments, segment_to_remove);
        }
        else
        {
            return;
        }
    }

    // Inserts a node from the tree, rebalancing when needed. Also finds the left and right neighbour of the removed node.
    template<typename SegmentT>
    void Sweep_Line_Status_structure<SegmentT>::remove(const std::vector<SegmentT>& segments, const int segment_to_remove, int& left_node, int& right_node)
    {
        if (root != nullptr)
        {
            left_node = root.get()->get_left_neighbour(segments, current_event_point);
            right_node = root.get()->get_right_neighbour(segments, current_event_point);

            root = remove(std::move(root), segments, segment_to_remove);
        }
        else
        {
            return;
        }
    }

    template<typename SegmentT>
    std::unique_ptr<typename Sweep_Line_Status_structure<SegmentT>::Node> Sweep_Line_Status_structure<SegmentT>::remove(std::unique_ptr<Node>&& node, const std::vector<SegmentT>& segments, const int segment_to_remove)
    {
        //TODO: This remove function is a bit order sensitive.. if two segments lie on the same line it defaults to right?? Use order function like in insert?
        std::unique_ptr<Node> new_root = std::move(node);

        Float current_x_position = segments.at(segment_to_remove).y_intersect(current_event_point.y);

        if (segment_to_remove == new_root->segment)
        {
            if (new_root->left == nullptr)
            {
                if (new_root->right != nullptr)
                {
                    new_root->right->parent = new_root->parent;
                }
                //Simply return the right subtree
                return std::move(new_root->right);
            }

            //Find most right of left subtree
            Node* child = new_root->left.get();
            while (child->right != nullptr)
            {
                child = child->right.get();
            }

            //Hoist the segment from the left child to the new root node and remove it from left (balancing in the process)
            const int child_segment = child->segment;
            new_root->left = remove_from_parent(std::move(new_root->left), segments, child_segment);
            new_root->segment = child_segment;

            //Balance tree if necessary
            if (new_root->height_difference() == -2)
            {
                if (new_root->right->height_difference() <= 0)
                {
                    new_root = rotate_left(std::move(new_root));
                }
                else
                {
                    new_root = rotate_right_left(std::move(new_root));
                }
            }
        }
        else if (current_x_position <= segments.at(new_root->segment).y_intersect(current_event_point.y))
        {
            new_root->left = remove_from_parent(std::move(new_root->left), segments, segment_to_remove);

            if (new_root->height_difference() == -2)
            {
                if (new_root->right->height_difference() <= 0)
                {
                    new_root = rotate_left(std::move(new_root));
                }
                else
                {
                    new_root = rotate_right_left(std::move(new_root));
                }
            }
        }
        else
        {
            new_root->right = remove_from_parent(std::move(new_root->right), segments, segment_to_remove);

            if (new_root->height_difference() == 2)
            {
                if (new_root->left->height_difference() >= 0)
                {
                    new_root = rotate_right(std::move(new_root));
                }
                else
                {
                    new_root = rotate_left_right(std::move(new_root));
                }
            }
        }

        new_root->calculate_height();

        return new_root;
    }

    template<typename SegmentT>
    std::unique_ptr<typename Sweep_Line_Status_structure<SegmentT>::Node> Sweep_Line_Status_structure<SegmentT>::remove_from_parent(std::unique_ptr<Node>&& parent, const std::vector<SegmentT>& segments, const int segment_to_remove)
    {
        if (parent != nullptr)
        {
            return remove(std::move(parent), segments, segment_to_remove);
        }
        else
        {
            return nullptr;
        }
    }

    //TODO: returns true if another segment intersect at the same point.
    template<typename SegmentT>
    bool Sweep_Line_Status_structure<SegmentT>::contains(const std::vector<SegmentT>& segments, const SegmentT* search_segment)
    {
        Node* node = root.get();

        Float current_x_position = search_segment->y_intersect(current_event_point.y);

        while (node != nullptr)
        {
            Float y_intersect = segments[node->segment].y_intersect(current_event_point.y);

            if (current_x_position.is_inf())
            {
                //Horizontal
                if (current_x_position.is_inf())
                {
                    //TODO: This is a bandaid, fix: We should check equality and probably search in both left and right.
                    return true;
                }

                return current_event_point.x < current_x_position;
            }
            else if (current_x_position < y_intersect)
            {
                node = node->left.get();
            }
            else if (current_x_position > y_intersect)
            {
                node = node->right.get();
            }
            else
            {
                return true;
            }
        }

        return false;
    }

    template<typename SegmentT>
    typename Sweep_Line_Status_structure<SegmentT>::Node* Sweep_Line_Status_structure<SegmentT>::get_node(const std::vector<SegmentT>& segments, const Float& x_position) const
    {
        Node* node = root.get();

        while (node != nullptr)
        {
            Float y_intersect = segments[node->segment].y_intersect(current_event_point.y);

            if (y_intersect.is_inf())
            {
                //If infinite, horizontal segment, check if event point lies on the segment.
                if (x_position < segments[node->segment].get_left_point()->x)
                {
                    node = node->left.get();
                }
                else if (x_position > segments[node->segment].get_right_point()->x)
                {
                    node = node->right.get();
                }
                else
                {
                    return node;
                }
            }

            if (x_position < y_intersect)
            {
                node = node->left.get();
            }
            else if (x_position > y_intersect)
            {
                node = node->right.get();
            }
            else
            {
                return node;
            }
        }

        return nullptr;
    }

    template<typename SegmentT>
    std::unique_ptr<typename Sweep_Line_Status_structure<SegmentT>::Node> Sweep_Line_Status_structure<SegmentT>::rotate_left(std::unique_ptr<Node>&& old_root)
    {
        std::unique_ptr<Node> new_root = std::move(old_root->right);
        new_root->parent = old_root->parent;
        old_root->parent = new_root.get();

        old_root->right = std::move(new_root->left);
        if (old_root->right != nullptr)
        {
            old_root->right->parent = old_root.get();
        }
        new_root->left = std::move(old_root);

        new_root->left->calculate_height();

        return new_root;
    }

    template<typename SegmentT>
    std::unique_ptr<typename Sweep_Line_Status_structure<SegmentT>::Node> Sweep_Line_Status_structure<SegmentT>::rotate_right(std::unique_ptr<Node>&& old_root)
    {
        std::unique_ptr<Node> new_root = std::move(old_root->left);
        new_root->parent = old_root->parent;
        old_root->parent = new_root.get();

        old_root->left = std::move(new_root->right);
        if (old_root->left != nullptr)
        {
            old_root->left->parent = old_root.get();
        }
        new_root->right = std::move(old_root);

        new_root->right->calculate_height();

        return new_root;
    }

    template<typename SegmentT>
    std::unique_ptr<typename Sweep_Line_Status_structure<SegmentT>::Node> Sweep_Line_Status_structure<SegmentT>::rotate_right_left(std::unique_ptr<Node>&& old_root)
    {
        std::unique_ptr<Node> right = std::move(old_root->right);
        std::unique_ptr<Node> new_root = std::move(right->left);
        new_root->parent = old_root->parent;

        right->left = std::move(new_root->right);
        if (right->left != nullptr)
        {
            right->left->parent = right.get();
        }

        old_root->right = std::move(new_root->left);
        if (old_root->right != nullptr)
        {
            old_root->right->parent = old_root.get();
        }


        new_root->left = std::move(old_root);
        if (new_root->left != nullptr)
        {
            new_root->left->parent = new_root.get();
        }
        new_root->right = std::move(right);
        if (new_root->right != nullptr)
        {
            new_root->right->parent = new_root.get();
        }
        new_root->right->calculate_height();
        new_root->left->calculate_height();

        return new_root;
    }

    template<typename SegmentT>
    std::unique_ptr<typename Sweep_Line_Status_structure<SegmentT>::Node> Sweep_Line_Status_structure<SegmentT>::rotate_left_right(std::unique_ptr<Node>&& old_root)
    {
        std::unique_ptr<Node> left = std::move(old_root->left);
        std::unique_ptr<Node> new_root = std::move(left->right);
        new_root->parent = old_root->parent;

        left->right = std::move(new_root->left);
        if (left->right != nullptr)
        {
            left->right->parent = left.get();
        }


        old_root->left = std::move(new_root->right);
        if (old_root->left != nullptr)
        {
            old_root->left->parent = old_root.get();
        }
        new_root->right = std::move(old_root);
        if (new_root->right != nullptr)
        {
            new_root->right->parent = new_root.get();
        }
        new_root->left = std::move(left);
        if (new_root->left != nullptr)
        {
            new_root->left->parent = new_root.get();
        }
        new_root->left->calculate_height();
        new_root->right->calculate_height();

        return new_root;
    }

    // Sets the height of a subtree based on its child subtrees
    template<typename SegmentT>
    void Sweep_Line_Status_structure<SegmentT>::Node::calculate_height()
    {
        height = 0;

        if (left != nullptr)
        {
            height = std::max(height, left->height);
        }

        if (right != nullptr)
        {
            height = std::max(height, right->height);
        }

        height += 1;
    }

    // Calculates the difference in height between a nodes left and right subtree
    template<typename SegmentT>
    int Sweep_Line_Status_structure<SegmentT>::Node::height_difference()
    {
        int left_height = 0;
        int right_height = 0;

        if (left != nullptr)
        {
            left_height = left->height + 1;
        }

        if (right != nullptr)
        {
            right_height = right->height + 1;
        }

        return left_height - right_height;
    }

    // Get Left neighbour of the Segment not the node!
    template<typename SegmentT>
    int Sweep_Line_Status_structure<SegmentT>::Node::get_left_neighbour(const std::vector<SegmentT>& segments, const Vec2& current_event_point) const
    {
        //The left neighbour is the right most leaf in the left subtree or one of the parent nodes

        // Check if right pointer of left node is empty
        if (left != nullptr)
        {
            const Node* current_left = left.get();

            //Traverse to the right most leaf node
            while (current_left->right != nullptr)
            {
                current_left = current_left->right.get();
            }

            return current_left->segment;
        }

        //No left subtree, one of the parents could be the left neighbour
        if (parent != nullptr)
        {
            //TODO: Use event x here, also at the sister function.
            Float current_x_position = segments.at(segment).y_intersect(current_event_point.y);
            const Node* current_parent = parent;
            while (current_x_position <= segments.at(current_parent->segment).y_intersect(current_event_point.y))
            {
                current_parent = current_parent->parent;
                if (current_parent == nullptr)
                {
                    return -1;
                }
            }

            return current_parent->segment;
        }

        return -1;
    }

    template<typename SegmentT>
    int Sweep_Line_Status_structure<SegmentT>::Node::get_right_neighbour(const std::vector<SegmentT>& segments, const Vec2& current_event_point) const
    {
        //The right neighbour is the left most leaf in the right subtree

        // Check if left pointer of right node is empty
        if (right != nullptr)
        {
            const Node* current_right = right.get();

            //Traverse to the left most leaf node
            while (current_right->left != nullptr)
            {
                current_right = current_right->left.get();
            }

            return current_right->segment;
        }

        //No right subtree, one of the parents could be the right neighbour
        if (parent != nullptr)
        {
            Float current_x_position = segments.at(segment).y_intersect(current_event_point.y);
            const Node* current_parent = parent;
            while (current_x_position >= segments.at(current_parent->segment).y_intersect(current_event_point.y))
            {
                current_parent = current_parent->parent;
                if (current_parent == nullptr)
                {
                    return -1;
                }
            }

            return current_parent->segment;
        }

        return -1;
    }

    template<typename SegmentT>
    std::vector<int> Sweep_Line_Status_structure<SegmentT>::Node::get_all_neighbours(const std::vector<SegmentT>& segments, const Vec2& current_event_point, int& left_neighbour, int& right_neighbour
    ) const
    {
        //TODO: Cleanup please, we can probably just do this left to right with an inorder walk from the found root node.

        std::vector<int> ordered_intersecting_segments;

        std::vector<const Node*> neighbouring_nodes;
        neighbouring_nodes.push_back(this);

        if (left != nullptr)
        {
            //Get all nodes intersecting the same point, the last element returned is the most left node
            left->get_intersecting_right_to_left(segments, current_event_point, neighbouring_nodes);
        }

        //Set the first left neighbour that doesn't intersect (the left neighbour of the left most intersecting node)
        left_neighbour = neighbouring_nodes.back()->get_left_neighbour(segments, current_event_point);

        //Report the found segments in order from left to right
        for (auto node_it = neighbouring_nodes.rbegin(); node_it != neighbouring_nodes.rend(); ++node_it)
        {
            ordered_intersecting_segments.emplace_back((*node_it)->segment);
        }

        neighbouring_nodes.clear();

        if (right != nullptr)
        {
            //Get all nodes intersecting the same point, the last element returned is the most right node
            right->get_intersecting_left_to_right(segments, current_event_point, neighbouring_nodes);
        }

        if (!neighbouring_nodes.empty())
        {
            //Set the first right neighbour that doesn't intersect
            right_neighbour = neighbouring_nodes.back()->get_right_neighbour(segments, current_event_point);

            //Report the found segments in order from left to right
            for (const Node* node : neighbouring_nodes)
            {
                ordered_intersecting_segments.emplace_back(node->segment);
            }
        }
        else
        {
            //If there are no right neighbours, get the right neighbour of the root node
            right_neighbour = this->get_right_neighbour(segments, current_event_point);
        }

        return ordered_intersecting_segments;
    }

    template<typename SegmentT>
    bool Sweep_Line_Status_structure<SegmentT>::Node::get_intersecting_left_to_right(const std::vector<SegmentT>& segments, const Vec2& current_event_point, std::vector<const Node*>& right_nodes) const
    {
        //Perform an inorder tree walk from left to right, reporting intersecting nodes.

        if (left != nullptr)
        {
            //If left subtree contained a segment not on this point, early out.
            if (!left->get_intersecting_left_to_right(segments, current_event_point, right_nodes))
            {
                return false;
            }
        }

        //The segment should always intersect with the sweep line so we can simplify by checking the x-coordinate of the intersection
        //In the edge-case that the segment lies horizontal (there are infinite intersections) we check if the event point lies on or in between the endpoints
        if (Float y_intersect = segments[segment].y_intersect(current_event_point.y);
            current_event_point.x == y_intersect ||
            (y_intersect.is_inf() &&
                (segments[segment].get_left_point()->x <= current_event_point.x && current_event_point.x <= segments[segment].get_right_point()->x)))
        {
            right_nodes.emplace_back(this);
        }
        else
        {
            return false;
        }

        if (right != nullptr)
        {
            //If right subtree contained a segment not on this point, early out.
            if (!right->get_intersecting_left_to_right(segments, current_event_point, right_nodes))
            {
                return false;
            }
        }

        return true;
    }

    template<typename SegmentT>
    bool Sweep_Line_Status_structure<SegmentT>::Node::get_intersecting_right_to_left(const std::vector<SegmentT>& segments, const Vec2& current_event_point, std::vector<const Node*>& left_nodes) const
    {
        //Perform an inorder tree walk from right to left, reporting intersecting nodes.

        if (right != nullptr)
        {
            //If right subtree contained a segment not on this point, early out.
            if (!right->get_intersecting_right_to_left(segments, current_event_point, left_nodes))
            {
                return false;
            }
        }

        //The segment should always intersect with the sweep line so we can simplify by checking the x-coordinate of the intersection
        //In the edge-case that the segment lies horizontal (there are infinite intersections) we check if the event point lies on or in between the endpoints
        if (Float y_intersect = segments[segment].y_intersect(current_event_point.y);
            current_event_point.x == y_intersect ||
            (y_intersect.is_inf() &&
                (segments[segment].get_left_point()->x <= current_event_point.x && current_event_point.x <= segments[segment].get_right_point()->x)))
        {
            left_nodes.emplace_back(this);
        }
        else
        {
            return false;
        }

        if (left != nullptr)
        {
            //If left subtree contained a segment not on this point, early out.
            if (!left->get_intersecting_right_to_left(segments, current_event_point, left_nodes))
            {
                return false;
            }
        }

        return true;
    }

    template<typename SegmentT>
    std::vector<int> Sweep_Line_Status_structure<SegmentT>::get_all_nodes_on_point(const std::vector<SegmentT> segments, int& left_neighbour, int& right_neighbour) const
    {
        //Find the first root node containing a segment intersecting the given event point
        const Node* event_node = get_node(segments, current_event_point.x);

        if (event_node != nullptr)
        {
            //Find all the neighbouring nodes that contain segments that intersect the point as well, these are adjacent in the tree
            return event_node->get_all_neighbours(segments, current_event_point, left_neighbour, right_neighbour);
        }
        else
        {
            return std::vector<int>();
        }
    }

    //Returns a string containing a pretty printed tree
    template<typename SegmentT>
    std::string Sweep_Line_Status_structure<SegmentT>::print_tree() const
    {
        std::string tree_string = "";
        if (root != nullptr)
        {
            root->print_tree(root.get(), 0, tree_string);
        }

        return tree_string;
    }

    template<typename SegmentT>
    void Sweep_Line_Status_structure<SegmentT>::Node::print_tree(Node* root, int spacing, std::string& tree_string) const
    {
        if (root == nullptr)
        {
            return;
        }

        spacing += 12;

        print_tree(root->right.get(), spacing, tree_string);

        tree_string += '\n';

        for (size_t i = 12; i < spacing; i++)
        {
            tree_string += " ";
        }

        tree_string += std::to_string(root->segment);
        tree_string += '\n';

        print_tree(root->left.get(), spacing, tree_string);
    }
}