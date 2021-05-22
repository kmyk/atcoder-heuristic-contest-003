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

vector<pair<int, int>> solve_with_dijkstra(int sy, int sx, int ty, int tx, const array<array<int64_t, W - 1>, H>& hr, const array<array<int64_t, H - 1>, W>& vr) {
    reversed_priority_queue<tuple<int64_t, int, int>> que;
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

    // reconstruct
    vector<pair<int, int>> path;
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
    return path;
}

template <class RandomEngine>
vector<pair<int, int>> solve_with_random(int sy, int sx, int ty, int tx, RandomEngine& gen) {
    assert (sy <= ty);
    assert (sx <= tx);

    vector<pair<int, int>> path;
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
    return path;
}

vector<pair<int, int>> solve_with_scan(int sy, int sx, int ty, int tx, int scan_y) {
    assert (sy <= ty);
    assert (sx <= tx);
    assert (scan_y != sy and scan_y != ty);
    vector<pair<int, int>> path;
    int y = sy;
    int x = sx;
    path.emplace_back(y, x);
    while (x > 0) {
        x -= 1;
        path.emplace_back(y, x);
    }
    while (y != scan_y) {
        y += (y < scan_y ? 1 : -1);
        path.emplace_back(y, x);
    }
    while (x < W - 1) {
        x += 1;
        path.emplace_back(y, x);
    }
    while (y != ty) {
        y += (y < ty ? 1 : -1);
        path.emplace_back(y, x);
    }
    while (x > tx) {
        x -= 1;
        path.emplace_back(y, x);
    }
    return path;
}

template <class RandomEngine>
vector<pair<int, int>> solve1(int sy, int sx, int ty, int tx, const vector<pair<vector<pair<int, int>>, int64_t>>& history, RandomEngine& gen) {
    assert(sy <= ty);
    assert(sx <= tx);
    assert(ty - sy <= tx - sx);

    // estimate the base value of rows and cols
    vector<int64_t> base_row(H, INT64_MAX);
    vector<int64_t> base_col(W, INT64_MAX);
    for (auto [path, score] : history) {
        if (path.size() < H and path.size() < W) {
            continue;
        }
        vector<int> count_row(H);
        vector<int> count_col(H);
        REP (i, path.size() - 1) {
            auto [ay, ax] = path[i];
            auto [by, bx] = path[i + 1];
            if (bx == ax) {
                count_row[ax] += 1;
            } else if (by == ay) {
                count_col[ay] += 1;
            } else {
                assert (false);
            }
        }
        REP (y, H) {
            if (count_row[y] == W - 1) {
                base_row[y] = score / path.size();
            }
        }
        REP (x, W) {
            if (count_col[x] == H - 1) {
                base_col[x] = score / path.size();
            }
        }
    }

    // analyze history for each edge
    array<array<int64_t, W - 1>, H> sum_hr = {};
    array<array<int64_t, H - 1>, W> sum_vr = {};
    array<array<int, W - 1>, H> used_hr = {};
    array<array<int, H - 1>, W> used_vr = {};
    for (auto [path, score] : history) {
        REP (i, path.size() - 1) {
            auto [ay, ax] = path[i];
            auto [by, bx] = path[i + 1];
            if (bx == ax) {
                sum_hr[ax][min(ay, by)] += score / (path.size() - 1);
                used_hr[ax][min(ay, by)] += 1;
            } else if (by == ay) {
                sum_vr[ay][min(ax, bx)] += score / (path.size() - 1);
                used_vr[ay][min(ax, bx)] += 1;
            } else {
                assert (false);
            }
        }
    }

    // estimate costs
    array<array<int64_t, W - 1>, H> hr = {};
    REP (y, H) {
        REP (x, W - 1) {
            if (base_row[y] != INT64_MAX) {
                hr[y][x] = base_row[y];
            } else if (used_hr[y][x]) {
                hr[y][x] = sum_hr[y][x] / used_hr[y][x] + uniform_int_distribution<int>(0, 1000)(gen);
                hr[y][x] = 5000;
            } else {
                hr[y][x] = 5000;
            }
        }
    }
    array<array<int64_t, H - 1>, W> vr = {};
    REP (x, W) {
        REP (y, H - 1) {
            if (base_col[x] != INT64_MAX) {
                vr[x][y] = base_col[x];
            } else if (used_vr[x][y]) {
                vr[x][y] = sum_vr[x][y] / used_vr[x][y] + uniform_int_distribution<int>(0, 1000)(gen);
                vr[x][y] = 5000;
            } else {
                vr[x][y] = 5000;
            }
        }
    }

#ifdef VERBOSE
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
#endif  // VERBOSE

    if (sx < 10 and W - 10 <= tx) {
        vector<int> scan_ys;
        REP3 (y, max(0, sy - 3), min(H, ty + 3 + 1)) {
            if (y == sy or y == ty) {
                continue;
            }
            if (base_row[y] == INT64_MAX) {
                scan_ys.push_back(y);
            }
        }
        if (not scan_ys.empty()) {
            int scan_y = scan_ys.front();
            if (H - scan_ys.back() - 1  < scan_y) {
                scan_y = scan_ys.back();
            }
            return solve_with_scan(sy, sx, ty, tx, scan_y);
        }
    }

    return solve_with_dijkstra(sy, sx, ty, tx, hr, vr);
}

template <class RandomEngine>
void solve(function<tuple<int, int, int, int> ()> read, function<int64_t (const string&)> write, int K, RandomEngine& gen, chrono::high_resolution_clock::time_point clock_end) {
    chrono::high_resolution_clock::time_point clock_begin = chrono::high_resolution_clock::now();

    vector<pair<vector<pair<int, int>>, int64_t>> history;
    REP (query, K) {
        // input
        int sy, sx, ty, tx;
        tie(sy, sx, ty, tx) = read();
        vector<pair<int, int>> path;

        // make the relatie relation between s and t simple
        bool is_flipped_vr = false;
        auto flip_vr = [&]() {
            is_flipped_vr = not is_flipped_vr;
            sy = H - sy - 1;
            ty = H - ty - 1;
            REP (i, path.size()) {
                path[i].first = W - path[i].first - 1;
            }
        };
        bool is_flipped_hr = false;
        auto flip_hr = [&]() {
            is_flipped_hr = not is_flipped_hr;
            sx = W - sx - 1;
            tx = W - tx - 1;
            REP (i, path.size()) {
                path[i].second = W - path[i].second - 1;
            }
        };
        bool is_flipped_diag = false;
        auto flip_diag = [&]() {
            is_flipped_diag = not is_flipped_diag;
            swap(sx, sy);
            swap(tx, ty);
            REP (i, path.size()) {
                swap(path[i].first, path[i].second);
            }
        };
        if (sy > ty) {
            flip_vr();
        }
        if (sx > tx) {
            flip_hr();
        }
        if (ty - sy > tx - sx) {
            flip_diag();
        }

        // solve
        path = solve1(sy, sx, ty, tx, history, gen);

        // stop flipping
        if (is_flipped_diag) {
            flip_diag();
        }
        if (is_flipped_hr) {
            flip_hr();
        }
        if (is_flipped_vr) {
            flip_vr();
        }

        // output
        int64_t score = write(get_command_from_path(path));
#ifdef VERBOSE
        cerr << "(" << sy << ", " << sx << ") -> (" << ty << ", " << tx << "): " << score << endl;
#endif  // VERBOSE

        // record the result
        history.emplace_back(path, score);
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
