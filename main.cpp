#include <bits/stdc++.h>
#define REP(i, n) for (int i = 0; (i) < (int)(n); ++ (i))
#define REP3(i, m, n) for (int i = (m); (i) < (int)(n); ++ (i))
#define REP_R(i, n) for (int i = (int)(n) - 1; (i) >= 0; -- (i))
#define REP3R(i, m, n) for (int i = (int)(n) - 1; (i) >= (int)(m); -- (i))
#define ALL(x) std::begin(x), std::end(x)
using namespace std;

template <class T> using reversed_priority_queue = priority_queue<T, vector<T>, greater<T> >;

class xor_shift_128 {
public:
    typedef uint32_t result_type;
    xor_shift_128(uint32_t seed = 42) {
        set_seed(seed);
    }
    void set_seed(uint32_t seed) {
        a = seed = 1812433253u * (seed ^ (seed >> 30));
        b = seed = 1812433253u * (seed ^ (seed >> 30)) + 1;
        c = seed = 1812433253u * (seed ^ (seed >> 30)) + 2;
        d = seed = 1812433253u * (seed ^ (seed >> 30)) + 3;
    }
    uint32_t operator() () {
        uint32_t t = (a ^ (a << 11));
        a = b; b = c; c = d;
        return d = (d ^ (d >> 19)) ^ (t ^ (t >> 8));
    }
    static constexpr uint32_t max() { return numeric_limits<result_type>::max(); }
    static constexpr uint32_t min() { return numeric_limits<result_type>::min(); }
private:
    uint32_t a, b, c, d;
};

constexpr int H = 30;
constexpr int W = 30;

string get_command_from_path(const vector<pair<int, int>>& path) {
    string command;
    assert (not path.empty());
    REP (i, path.size() - 1) {
        auto [ay, ax] = path[i];
        auto [by, bx] = path[i + 1];
        if (by == ay - 1 and bx == ax) {
            command.push_back('U');
        } else if (by == ay + 1 and bx == ax) {
            command.push_back('D');
        } else if (by == ay and bx == ax + 1) {
            command.push_back('R');
        } else if (by == ay and bx == ax - 1) {
            command.push_back('L');
        } else {
            assert (false);
        }
    }
    return command;
}

const int DY[4] = {-1, 1, 0, 0};
const int DX[4] = {0, 0, 1, -1};

template <class RandomEngine>
void solve(function<tuple<int, int, int, int> ()> read, function<int64_t (const string&)> write, int K, RandomEngine& gen, chrono::high_resolution_clock::time_point clock_end) {
    chrono::high_resolution_clock::time_point clock_begin = chrono::high_resolution_clock::now();

    array<array<int64_t, W - 1>, H> acc_hr = {};
    array<array<int64_t, H - 1>, W> acc_vr = {};
    array<array<int, W - 1>, H> used_hr = {};
    array<array<int, H - 1>, W> used_vr = {};
    REP (query, K) {
        // input
        int sy, sx, ty, tx;
        tie(sy, sx, ty, tx) = read();
        vector<pair<int, int>> path;

        // make the relatie relation between s and t simple
        bool is_flipped_vr = false;
        auto flip_vr = [&]() {
            is_flipped_vr = not is_flipped_vr;
            REP (y, H) {
                reverse(ALL(acc_vr[y]));
                reverse(ALL(used_vr[y]));
            }
            sy = H - sy - 1;
            ty = H - ty - 1;
            REP (i, path.size()) {
                path[i].first = W - path[i].first - 1;
            }
        };
        bool is_flipped_hr = false;
        auto flip_hr = [&]() {
            is_flipped_hr = not is_flipped_hr;
            REP (x, W) {
                reverse(ALL(acc_hr[x]));
                reverse(ALL(used_hr[x]));
            }
            sx = W - sx - 1;
            tx = W - tx - 1;
            REP (i, path.size()) {
                path[i].second = W - path[i].second - 1;
            }
        };
        if (sy > ty) {
            flip_vr();
        }
        if (sx > tx) {
            flip_hr();
        }
        assert (sy <= ty);
        assert (sx <= tx);

        // estimate costs
        array<array<double, W - 1>, H> hr = {};
        REP (y, H) {
            int used_row = accumulate(ALL(used_hr[y]), 0);
            int64_t acc_row = accumulate(ALL(acc_hr[y]), 0ll);
            REP (x, W - 1) {
                hr[y][x] = used_row == 0 ? 4000 : (5 * acc_hr[y][x] + acc_row) / (5 * used_hr[y][x] + used_row);
                hr[y][x] += uniform_int_distribution<int>(0, 300)(gen);
            }
        }
        array<array<double, H - 1>, W> vr = {};
        REP (x, W) {
            int used_col = accumulate(ALL(used_vr[x]), 0);
            int64_t acc_col = accumulate(ALL(acc_vr[x]), 0ll);
            REP (y, H - 1) {
                vr[x][y] = used_col == 0 ? 4000 : (5 * acc_vr[x][y] + acc_col) / (5 * used_vr[x][y] + used_col);
                vr[x][y] += uniform_int_distribution<int>(0, 300)(gen);
            }
        }
#ifdef VERBOSE
        if (query == K - 1) {
            cerr << "hr" << endl;;
            REP (y, H) {
                REP (x, W - 1) {
                    cerr << hr[y][x] << ' ';
                }
                cerr << endl;
            }
            cerr << "vr" << endl;;
            REP (x, W) {
                REP (y, H - 1) {
                    cerr << vr[x][y] << ' ';
                }
                cerr << endl;
            }
        }
#endif  // VERBOSE

        // construct the path
        // dijkstra
        reversed_priority_queue<tuple<double, int, int> > que;
        array<array<double, W>, H> dist;
        array<array<pair<int, int>, W>, H> parent;
        REP (y, H) {
            fill(ALL(dist[y]), INT64_MAX);
            fill(ALL(parent[y]), make_pair(-1, -1));
        }
        que.emplace(0, sy, sx);
        dist[sy][sx] = 0;
        while (not que.empty()) {
            auto [dist_y_x, y, x] = que.top();
            que.pop();
            if (dist[y][x] < dist_y_x) {
                continue;
            }
            REP (i, 4) {
                int ny = y + DY[i];
                int nx = x + DX[i];
                if (ny < 0 or H <= ny or nx < 0 or W <= nx) {
                    continue;
                }
                int64_t ndist = dist[y][x] + (DY[i] ?  vr[x][min(y, ny)] : hr[y][min(x, nx)]);
                if (ndist < dist[ny][nx]) {
                    dist[ny][nx] = ndist;
                    parent[ny][nx] = make_pair(y, x);
                    que.emplace(ndist, ny, nx);
                }
            }
        }
        int y = ty;
        int x = tx;
        path.emplace_back(y, x);
        while (parent[y][x] != make_pair(-1, -1)) {
            tie(y, x) = parent[y][x];
            path.emplace_back(y, x);
        }
        assert (y == sy);
        assert (x == sx);
        reverse(ALL(path));
#if 0
        // random
        assert (sy <= sy);
        assert (sx <= tx);
        int y = sy;
        int x = sx;
        path.emplace_back(y, x);
        while (y < ty and x < tx) {
            int r = uniform_int_distribution<int>(0, ty - y + tx - x - 1)(gen);
            if (r < ty - y) {
                y += 1;
                path.emplace_back(y, x);
            } else {
                x += 1;
                path.emplace_back(y, x);
            }
        }
        while (y < ty) {
            y += 1;
            path.emplace_back(y, x);
        }
        while (x < tx) {
            x += 1;
            path.emplace_back(y, x);
        }
#endif

        // stop flipping
        if (is_flipped_vr) {
            flip_vr();
        }
        if (is_flipped_hr) {
            flip_hr();
        }

        // output
        int64_t score = write(get_command_from_path(path));
#ifdef VERBOSE
        cerr << "(" << sy << ", " << sx << ") -> (" << ty << ", " << tx << "): " << score << endl;
#endif  // VERBOSE

        // record the result and update estimation
        REP (i, path.size() - 1) {
            auto [ay, ax] = path[i];
            auto [by, bx] = path[i + 1];
            if (bx == ax) {
                acc_hr[ax][min(ay, by)] += score / (path.size() - 1);
                used_hr[ax][min(ay, by)] += 1;
            } else if (by == ay) {
                acc_vr[ay][min(ax, bx)] += score / (path.size() - 1);
                used_vr[ay][min(ax, bx)] += 1;
            } else {
                assert (false);
            }
        }
    }
}

int main() {
    constexpr auto TIME_LIMIT = chrono::milliseconds(2000);
    chrono::high_resolution_clock::time_point clock_begin = chrono::high_resolution_clock::now();
    xor_shift_128 gen(20210425);
    auto read = [&]() {
        int sy, sx, ty, tx;
        cin >> sy >> sx >> ty >> tx;
        return make_tuple(sy, sx, ty, tx);
    };
    auto write = [&](const string& command) {
        cout << command << '\n';
        cout.flush();
        int64_t dist;
        cin >> dist;
        return dist;
    };
    constexpr int K = 1000;
    solve(read, write, K, gen, clock_begin + chrono::duration_cast<chrono::milliseconds>(TIME_LIMIT * 0.95));
    return 0;
}
