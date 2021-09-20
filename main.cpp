#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <set>
#include <cstdio>
#include <algorithm>
#include <iterator>
#include <random>

namespace city
{
	uint64_t gen(const uint64_t n)
	{
		std::random_device rd;  //Will be used to obtain a seed for the random number engine
		std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
		std::uniform_int_distribution<> distrib(0, n);
		return distrib(gen);
	}

	struct uint2 
	{
		uint64_t x, y;
		uint2() : x(0), y(0) {};
		uint2(uint64_t _x, uint64_t _y): x(_x), y(_y) {}
		inline uint2 operator+(const uint2 z) { return uint2(x + z.x, y + z.y);}
		inline uint2 operator-(const uint2 z) { return uint2(x - z.x, y - z.y); }
		inline uint2 operator*(const uint64_t z) { return uint2(x * z, y * z); }
		inline void operator=(const uint2 z) { x = z.x; y = z.y;}
		inline void operator+=(const uint2 z) { x += z.x; y += z.y; };
		inline void operator-=(const uint2 z) { x -= z.x; y -= z.y; };
		inline void operator*=(const uint64_t z) { x *= z; y *= z; };
		inline bool operator==(const uint2 z) { return ((x == z.x) * (y == z.y)); }
	};

	struct task
	{
		uint2 begin, end;
		task(){ begin = uint2(0, 0); end = uint2(0, 0); };
		task(uint2 _begin, uint2 _end) {begin = _begin;  end = _end;}
		task(uint64_t x0, uint64_t y0, uint64_t x1, uint64_t y1) 
		{
			begin = uint2(x0, y0);
			end = uint2(x1, y1);
		}
	};

	struct robot
	{
		bool isBusy;
		uint2 current_pos;
		task current_task;
		uint64_t** navigation_map = nullptr;
		uint64_t size_map;
		char states[60];

		enum state
		{
			STAY, UP, DOWN, LEFT, RIGHT, TRAP, POP
		} current_state = state::STAY;

		void create(const uint2 cur_pos, const uint64_t _size_map)
		{
			current_pos = cur_pos;
			size_map = _size_map;
			if (!navigation_map)
			{
				navigation_map = new uint64_t*[size_map];
				for (uint64_t i = 0; i < size_map; i++)
					navigation_map[i] = new uint64_t[size_map];
			}
			isBusy = false;
		}

		void destroy()
		{
			if (navigation_map)
			{
				for (uint64_t i = 0; i < size_map; i++)
					delete[] navigation_map[i];
				delete[] navigation_map;
				navigation_map = nullptr;
			}
		}

		~robot() { destroy(); }

		bool check_begin_task()
		{
			return (current_task.begin == current_pos);
		}

		bool check_end_task()
		{
			return (current_task.end == current_pos);
		}

		void save_state(uint32_t time) 
		{
			switch(current_state)
			{
			case state::STAY:
				states[time] = 'S';
				break;
			case state::DOWN:
				states[time] = 'D';
				break;
			case state::UP:
				states[time] = 'U';
				break;
			case state::LEFT:
				states[time] = 'L';
				break;
			case state::RIGHT:
				states[time] = 'R';
				break;
			case state::TRAP:
				states[time] = 'T';
				break;
			case state::POP:
				states[time] = 'P';
				break;
			}; 
		};

		void dump_states() const
		{
			for (uint16_t i = 0 ; i < 60; i++)
			 printf("%c", states[i]);
			printf("\n");
		}

		void move(uint32_t time) 
		{
			current_state = state::STAY;
			if (isBusy)
			{
				const auto& current_mark = navigation_map[current_pos.x][current_pos.y];
				if (current_mark == 1) 
				{
					if (check_begin_task())
					{
						current_state = state::TRAP;
						save_state(time);
						return;
					}
	//				if (check_end_task())
					{
						current_state = state::POP;
						isBusy = false;
						save_state(time);
						return;
					}
				}
				if (current_pos.x > 0)
				{
					const auto& neibour_mark = navigation_map[current_pos.x - 1][current_pos.y];
					if ((neibour_mark < current_mark) && (neibour_mark > 0))
					{
						current_pos = uint2(current_pos.x - 1, current_pos.y);
						current_state = state::UP;
						save_state(time);
						return;
					}
				}
				if (current_pos.y > 0)
				{
					const auto& neibour_mark = navigation_map[current_pos.x][current_pos.y - 1];
					if ((neibour_mark < current_mark) && (neibour_mark > 0))
					{
						current_pos = uint2(current_pos.x, current_pos.y - 1);
						current_state = state::LEFT;
						save_state(time);
						return;
					}
				}
				if (current_pos.x < (size_map - 1))
				{
					const auto& neibour_mark = navigation_map[current_pos.x + 1][current_pos.y];
					if ((neibour_mark < current_mark) && (neibour_mark > 0))
					{
						current_pos = uint2(current_pos.x + 1, current_pos.y);
						current_state = state::DOWN;
						save_state(time);
						return;
					}
				}
				if (current_pos.y < (size_map - 1))
				{
					const auto& neibour_mark = navigation_map[current_pos.x][current_pos.y+1];
					if ((neibour_mark < current_mark) && (neibour_mark > 0))
					{
						current_pos = uint2(current_pos.x, current_pos.y + 1);
						current_state = state::RIGHT;
						save_state(time);
						return;
					}
				}
			}
			save_state(time);
		}

		void print_navigation_map() 
		{
			std::cout << std::endl << "NAVIGATION MAP:" << std::endl << std::flush;
			for (uint64_t ii = 0; ii < size_map; ii++)
			{
				for (uint64_t ij = 0; ij < size_map; ij++)
					std::cout << navigation_map[ii][ij] << ' ';
				std::cout << std::endl;
			}
		}
	};

	class map_manager
	{
		bool** map = nullptr;
	public:
		uint64_t size = 0;

		void create()
		{
			if (!map && size > 0)
			{
				map = new bool*[size];
				for (uint64_t i = 0; i < size; i++)
					map[i] = new bool[size];
			}

			char cell;
			for (uint64_t i = 0; i < size; ++i)
				for(uint64_t j = 0; j < size; ++j)
			{
				std::cin >> cell;
				map[i][j] = (cell == '#') ? 1 : 0;
			}
		}

		bool get_cell(uint2 coord) const
		{
			return map[coord.x][coord.y];
		}

		void destroy()
		{
			if (map)
			{
				for (uint64_t i = 0; i < size; i++)
					delete[] map[i];
				delete[] map;
				map = nullptr;
			}
		}

		~map_manager() { destroy(); }

		void create_navigation(uint2 attraction_point, uint64_t** navigation) 
		{
			for (uint64_t ii = 0; ii < size; ++ii)
				for (uint64_t ij = 0; ij < size; ++ij)
					navigation[ii][ij] = 0;
			std::queue<uint2> neibours;
			navigation[attraction_point.x][attraction_point.y] = 1;
			neibours.push(attraction_point);
			while(!neibours.empty())
			{
				const uint2 current_cell = neibours.front();
				const auto& current_mark = navigation[current_cell.x][current_cell.y];
				neibours.pop();
				if(current_cell.x > 0)
				{
					auto& neibour_mark = navigation[current_cell.x - 1][current_cell.y];
					if (neibour_mark == 0 && !map[current_cell.x - 1][current_cell.y])
					{
						neibour_mark = current_mark + 1;
						neibours.push(uint2(current_cell.x - 1, current_cell.y));
					}
				}
				if(current_cell.y > 0) 
				{
					auto& neibour_mark = navigation[current_cell.x][current_cell.y - 1];
					if (neibour_mark == 0 && !map[current_cell.x][current_cell.y - 1])
					{
						neibour_mark = current_mark + 1;
				 		neibours.push(uint2(current_cell.x, current_cell.y - 1));
					}
				}
				if(current_cell.x < (size - 1))
				{
					auto& neibour_mark = navigation[current_cell.x + 1][current_cell.y];
					if (neibour_mark == 0 && !map[current_cell.x + 1][current_cell.y])
					{
						neibour_mark = current_mark + 1;
						neibours.push(uint2(current_cell.x + 1, current_cell.y));
					}
				}
				if(current_cell.y < (size - 1)) 
				{
					auto& neibour_mark = navigation[current_cell.x][current_cell.y + 1];
					if (neibour_mark == 0 && !map[current_cell.x][current_cell.y + 1])
					{
						neibour_mark = current_mark + 1;
						neibours.push(uint2(current_cell.x, current_cell.y + 1));
					}
				}
			}//while 
		}

		void print()
		{
			std::cout << std::endl << "MAP:" << std::endl << std::flush;
			for (uint64_t i = 0; i < size; ++i)
			{
				for (uint64_t j = 0; j < size; ++j)
				{
					std::cout << map[i][j] << std::flush;
				}
				std::cout << std::endl;
			}
		}
	};
	

	class task_manager 
	{
		std::multimap <uint64_t, task> tasks;
		std::multimap <uint64_t, task>::iterator it;
	public:
		uint64_t map_size = 0;
		uint64_t total = 0;
		uint64_t MaxTips = 0;

		inline uint64_t calc_key(const uint2 pos)
		{
			return pos.x + pos.y * map_size;
		}

		task find(uint2 pos)
		{
			uint64_t cur_key = calc_key(pos);
			auto it1 = std::find_if(tasks.rbegin(), tasks.rend(), [cur_key](const auto& pair) {return (cur_key < pair.first);}).base();
			auto it2 = std::find_if(tasks.cbegin(), tasks.cend(), [cur_key](const auto& pair) {return (cur_key > pair.first);});
			
			if (it2 == tasks.end())	it2--;
			if (it1 == tasks.end()) it1--;

			cur_key = fabs(it1->first - cur_key) < fabs(it2->first - cur_key) ? it1->first : it2->first;
			it = tasks.lower_bound(cur_key);
			return it->second;
		}
	
		void erase() 
		{
			if(it!=tasks.end())
			{
				tasks.erase(it);
				it = tasks.end();
			}
			
		};

		void insert(const task T) 
		{
			tasks.insert(std::pair<uint64_t, task>(calc_key(T.begin), T));
		}

		bool empty() { return tasks.empty(); }
	};

	class robot_manager
	{
	public:
		std::vector<robot> robots;
		task_manager tasks;
		map_manager map;
	
		uint64_t Cost_c = 0;
		uint64_t numIters = 0;
		uint64_t TotalTips = 0;

		void build_robots()
		{
			tasks.map_size = map.size;
			uint64_t numRobots = std::min((uint64_t)7, (((tasks.MaxTips - std::min(numIters * 60, map.size * map.size)) * tasks.total) / Cost_c));
			printf("%llu\n", numRobots);
			robots.resize(numRobots);
			for (uint64_t ir = 0 ; ir < numRobots; ++ir)
			{
				uint2 cur_pos(gen(map.size - 1), gen(map.size - 1));
				while (map.get_cell(cur_pos))
				{
					cur_pos = uint2(gen(map.size - 1), gen(map.size - 1));
				}
			//	if (ir == 0)cur_pos = uint2(3, 2);
			//	if (ir == 1)cur_pos = uint2(1, 0);
				robots[ir].create(cur_pos, map.size);
				if (ir < numRobots-1)
					printf("%llu %llu ", (cur_pos.x + 1), (cur_pos.y + 1));
				else
					printf("%llu %llu", (cur_pos.x + 1), (cur_pos.y + 1));
			}
			printf("\n");
		}

		void run()
		{
			for (uint64_t current_iter = 0; current_iter < numIters; ++current_iter)
			{
				uint64_t iQ;
				std::cin >> iQ;
				for (uint64_t iq = 0; iq < iQ; iq++)
				{
					uint64_t x0, y0, x1, y1;
					std::cin >> x0 >> y0 >> x1 >> y1;
					tasks.insert(task(--x0,--y0,--x1,--y1));
				}

				for (uint32_t time = 0; time < 60; time++)
				{
					for (auto& r : robots)
					{
			//			if (current_tip > 0 && r.isBusy) current_tip--;
						if (!r.isBusy && !tasks.empty())
						{
							r.current_task = tasks.find(r.current_pos);
			//				current_tip += tasks.MaxTips;
							if (r.current_pos == r.current_task.begin)
							{
								r.navigation_map[r.current_pos.x][r.current_pos.y] = 1;
							}
							else
							{
								map.create_navigation(r.current_task.begin, r.navigation_map);
							}
						//	if (r.navigation_map[r.current_pos.x][r.current_pos.y] != 0)
							{
						//		std::cout << "isBusy : "<< r.isBusy << "\ttask = " << r.current_task.begin.x + 1 << ' ' << r.current_task.begin.y + 1 << " : " << r.current_task.end.x + 1 << ' ' << r.current_task.end.y + 1 << "\n";
								r.isBusy = true;
								tasks.erase();
							}
						}
						r.move(time);
						if (r.current_state == robot::state::TRAP) 
						{
							map.create_navigation(r.current_task.end, r.navigation_map);
						}
			//			if (r.current_state == robot::state::POP)
			//			{
			//				TotalTips += current_tip;
			//				current_tip = 0;
			//			}
					}
				}
				for (const auto& r : robots)
				{
					r.dump_states();
				//	std::cout << r.current_pos.x + 1 << ' ' << r.current_pos.y + 1 << " | TotalTips = " << TotalTips - Cost_c * numRobots << std::endl;
				}

			}
		};
		void destroy_robots() 
		{
			for (auto& r : robots) 
			{
				r.destroy();
			}
		}
	};
};

int main() 
{
	city::robot_manager petya;
	std::cin >> petya.map.size >> petya.tasks.MaxTips >> petya.Cost_c;
	petya.map.create();
	std::cin >> petya.numIters >> petya.tasks.total;
	petya.build_robots();
	petya.run();
	petya.map.destroy();
	petya.destroy_robots();
#ifdef _WIN32 
	system("pause");
#endif
	return 0;
}