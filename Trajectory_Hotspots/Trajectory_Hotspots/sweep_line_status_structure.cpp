#include "pch.h"
#include "sweep_line_status_structure.h"

namespace Segment_Intersection_Sweep_Line
{
    /// <summary>
    /// Inserts a new node into the tree, rebalancing when needed.
    /// </summary>
    /// <typeparam name="T"></typeparam>
    /// <param name="segment"></param>
    void Sweep_Line_Status_structure::insert(const Segment* new_segment, const Segment*& left_node, const Segment*& right_node)
    {
        if (root != nullptr)
        {
            const Node* node = nullptr;

            root = insert(std::move(root), new_segment, node);

            if (node != nullptr)
            {
                left_node = node->get_left_neighbour(line_position);
                right_node = node->get_right_neighbour(line_position);
            }
            // get neighbours
            //left_node = 
            //right_node = 
        }
        else
        {
            root = std::make_unique<Node>(new_segment);
        }
    }

    std::unique_ptr<typename Sweep_Line_Status_structure::Node> Sweep_Line_Status_structure::insert(std::unique_ptr<Node>&& node, const Segment* new_segment, const Node*& added_node)
    {
        std::unique_ptr<Node> new_root = std::move(node);
        //TODO: maybe put in a function returns which is smaller
        float current_x_position = new_segment->y_intersect(line_position);

        if (current_x_position <= new_root->segment->y_intersect(line_position))
        {
            new_root->left = add_to_subtree(std::move(new_root->left), new_segment, added_node, new_root.get());

            if (new_root->height_difference() == 2)
            {
                if (current_x_position <= new_root->left->segment->y_intersect(line_position))
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
            new_root->right = add_to_subtree(std::move(new_root->right), new_segment, added_node, new_root.get());

            if (new_root->height_difference() == -2)
            {
                if (current_x_position > new_root->right->segment->y_intersect(line_position))
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

    std::unique_ptr<typename Sweep_Line_Status_structure::Node> Sweep_Line_Status_structure::add_to_subtree(std::unique_ptr<Node>&& root, const Segment* new_segment, const Node*& added_node, const Node* parent)
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
            return insert(std::move(root), new_segment, added_node);
        }
    }

    void Sweep_Line_Status_structure::remove(const Segment* segment_to_remove)
    {
        if (root != nullptr)
        {
            root = remove(std::move(root), segment_to_remove);
        }
        else
        {
            return;
        }
    }

    std::unique_ptr<typename Sweep_Line_Status_structure::Node> Sweep_Line_Status_structure::remove(std::unique_ptr<Node>&& node, const Segment* segment_to_remove)
    {
        std::unique_ptr<Node> new_root = std::move(node);

        float current_x_position = segment_to_remove->y_intersect(line_position);

        if (*segment_to_remove == *new_root->segment)
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
            std::unique_ptr<Node> child = std::move(new_root->left);
            while (child->right != nullptr)
            {
                child = std::move(child->right);
            }

            //Hoist the segment from the left child to the new root node and remove it from left (balancing in the process)
            const Segment* child_segment = child->segment;
            new_root->left = remove_from_parent(std::move(new_root->left), child_segment);
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
        else if (current_x_position < new_root->segment->y_intersect(line_position))
        {
            new_root->left = remove_from_parent(std::move(new_root->left), segment_to_remove);

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
            new_root->right = remove_from_parent(std::move(new_root->right), segment_to_remove);

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

    std::unique_ptr<typename Sweep_Line_Status_structure::Node> Sweep_Line_Status_structure::remove_from_parent(std::unique_ptr<Node>&& parent, const Segment* segment_to_remove)
    {
        if (parent != nullptr)
        {
            return remove(std::move(parent), segment_to_remove);
        }
        else
        {
            return nullptr;
        }
    }

    bool Sweep_Line_Status_structure::contains(const Segment* search_segment)
    {
        Node* node = root.get();

        float current_x_position = search_segment->y_intersect(line_position);

        while (node != nullptr)
        {
            if (current_x_position < node->segment->y_intersect(line_position))
            {
                node = node->left.get();
            }
            else if (current_x_position > node->segment->y_intersect(line_position))
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


    std::unique_ptr<typename Sweep_Line_Status_structure::Node> Sweep_Line_Status_structure::rotate_left(std::unique_ptr<Node>&& old_root)
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

    std::unique_ptr<typename Sweep_Line_Status_structure::Node> Sweep_Line_Status_structure::rotate_right(std::unique_ptr<Node>&& old_root)
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

    std::unique_ptr<typename Sweep_Line_Status_structure::Node> Sweep_Line_Status_structure::rotate_right_left(std::unique_ptr<Node>&& old_root)
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

    std::unique_ptr<typename Sweep_Line_Status_structure::Node> Sweep_Line_Status_structure::rotate_left_right(std::unique_ptr<Node>&& old_root)
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
    void Sweep_Line_Status_structure::Node::calculate_height()
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
    int Sweep_Line_Status_structure::Node::height_difference()
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
    // Get Left neighbour of the Segment and not the node!
    const Segment* Sweep_Line_Status_structure::Node::get_left_neighbour(const float line_position) const
    {
        // voor linker neighbour grootste getal in de linker kant van root
        // voor de rechter neighbour kleinste getal in de rechter kan van root
        // als groot
        // 


        // Check if right pointer of right node is empty or not
        if (right != nullptr)
        {
            const Node* current_right = right.get();
            while (current_right->right != nullptr)
            {
                current_right = current_right->right.get();
                if (current_right == nullptr)
                {
                    return nullptr;
                }
            }
            return current_right->segment;
        }
        // Check if right pointer of left node is empty or not
        if (left != nullptr)
        {
            const Node* current_right = left->right.get();
            while (current_right->left != nullptr)
            {
                current_right = current_right->right.get();
                if (current_right == nullptr)
                {
                    return nullptr;
                }
            }
            return current_right->segment;
        }
        if (parent != nullptr)
        {
            float current_x_position = segment->y_intersect(line_position);
            const Node* current_parent = parent;
            while (current_x_position < current_parent->segment->y_intersect(line_position))
            {
                current_parent = current_parent->parent;
                if (current_parent == nullptr)
                {
                    return nullptr;
                }
            }
            return current_parent->segment;
        }



        return nullptr;
    }

    const Segment* Sweep_Line_Status_structure::Node::get_right_neighbour(const float line_position) const
    {
        if (right != nullptr)
        {
            const Node* current_right = right.get();
            while (current_right->right != nullptr)
            {
                current_right = current_right->right.get();
                if (current_right == nullptr)
                {
                    return nullptr;
                }
            }
            return current_right->segment;
        }
        // Check if right pointer of left node is empty or not
        if (left != nullptr)
        {
            const Node* current_right = left->right.get();
            while (current_right->left != nullptr)
            {
                current_right = current_right->right.get();
                if (current_right == nullptr)
                {
                    return nullptr;
                }
            }
            return current_right->segment;
        }
        if (parent != nullptr)
        {
            float current_x_position = segment->y_intersect(line_position);
            const Node* current_parent = parent;
            while (current_x_position > current_parent->segment->y_intersect(line_position))
            {
                current_parent = current_parent->parent;
                if (current_parent == nullptr)
                {
                    return nullptr;
                }
            }
            return current_parent->segment;
        }



        return nullptr;
    }
}