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
vector<pair<int, int>> solve1(int sy, int sx, int ty, int tx, const vector<pair<vector<pair<int, int>>, int64_t>>& history, RandomEngine& gen) {
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
        if (sy > ty) {
            flip_vr();
        }
        if (sx > tx) {
            flip_hr();
        }

        // solve
        path = solve1(sy, sx, ty, tx, history, gen);

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
