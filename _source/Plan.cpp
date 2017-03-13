#include "Plan.h"

void _plan(const int &type, const Trajectory::State &state_i, const Trajectory::State &state_g, const double &accel, const double l, const double w, vector<Trajectory::State> *adjust_states_front, vector<Trajectory::State> *adjust_states_end)
{
	auto start = steady_clock::now();
	RRT::RTG_RRT tree(state_i, state_g);
	int result = tree.search(type, l, w);
	Trajectory::traj trajectory(state_i, state_g, adjust_states_front);
	if (result != 0)
	{
		tree.getpath(tree._dest()->_index());
		trajectory._state(result, accel, *tree._route_tree(), adjust_states_end);
		tree._motion(*trajectory._state_now());

		return;
	}

	RRT::RTG_RRT original_tree;
	vector<RRT::Index> *NN = new vector<RRT::Index>;
	RRT::Index new_root;
	bool search = true;
	while (search == true)
	{
		original_tree._copy(tree);
		tree.near_search(*tree._dest()->_node(), NN);
		new_root = *NN->rbegin();
		NN->pop_back();
		tree.reset(new_root);
		result = tree.search(0);
		while (tree._tree()->size() == 1 && result == 0)
		{
			if (!NN->empty())
			{
				new_root = *NN->rbegin();
				NN->pop_back();
			}
			else
			{
				search = false;
				break;
			}
			tree._copy(original_tree);
			tree.reset(new_root);
			result = tree.search(0);
		}

		original_tree.getpath(new_root);
		if (original_tree._route_tree()->size() == 1)
			continue;
		trajectory._state(result, accel, *original_tree._route_tree(), adjust_states_end);
		tree._motion(*trajectory._state_now());

		if (result == 1)
			break;
	}
	tree.getpath();
	trajectory._state(result, accel, *tree._route_tree(), adjust_states_end);
	tree._motion(*trajectory._state_now());

	delete NN;
	return;
}