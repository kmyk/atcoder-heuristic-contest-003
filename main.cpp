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
            int64_t ndist = dist[y][x] + (DY[i] ? vr[x][min(y, ny)] : hr[y][min(x, nx)]);
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

array<array<int64_t, W - 1>, H> make_costs_from_base(const array<int64_t, H>& row) {
    array<array<int64_t, W - 1>, H> hr;
    REP (y, H) {
        fill(ALL(hr[y]), row[y]);
    }
    return hr;
}

template <class RandomEngine>
array<array<int64_t, W - 1>, H> make_costs_from_base_with_random(const array<int64_t, H>& row, RandomEngine& gen) {
    array<array<int64_t, W - 1>, H> hr;
    REP (y, H) {
        REP (x, W - 1) {
            hr[y][x] = row[y] + uniform_int_distribution<int>(0, 300)(gen);
        }
    }
    return hr;
}

class path_composer {
    vector<pair<int, int>> moves;

public:
    path_composer(int sy, int sx) {
        assert (0 <= sy and sy < H);
        assert (0 <= sx and sx < W);
        moves.emplace_back(sy, sx);
    }

    vector<pair<int, int>> operator *() const {
        vector<pair<int, int>> path;
        for (auto [ty, tx] : moves) {
            if (path.empty()) {
                path.emplace_back(ty, tx);
            }
            auto [y, x] = path.back();
            if (y == ty) {
                while (x != tx)  {
                    x += (x < tx ? 1 : -1);
                    path.emplace_back(y, x);
                }
            } else if (x == tx) {
                while (y != ty)  {
                    y += (y < ty ? 1 : -1);
                    path.emplace_back(y, x);
                }
            } else {
                assert (false);
            }
        }
        return path;
    }

    // A `moves` is the sequence of the corners of a `path`.
    vector<pair<int, int>> get_moves() const {
        return moves;
    }

    operator bool() const {
        array<array<bool, W>, H> used = {};
        for (auto [y, x] : **this) {
            if (used[y][x]) {
                return false;
            }
            used[y][x] = true;
        }
        return true;
    }

    void go_up() {
        auto [y, x] = moves.back();
        assert (y - 1 >= 0);
        moves.emplace_back(y - 1, x);
    }

    void go_down() {
        auto [y, x] = moves.back();
        assert (y + 1 < H);
        moves.emplace_back(y + 1, x);
    }

    void go_right() {
        auto [y, x] = moves.back();
        assert (x + 1 < W);
        moves.emplace_back(y, x + 1);
    }

    void go_left() {
        auto [y, x] = moves.back();
        assert (x - 1 >= 0);
        moves.emplace_back(y, x - 1);
    }

    void go_row(int ty) {
        assert (0 <= ty and ty < H);
        auto [y, x] = moves.back();
        moves.emplace_back(ty, x);
    }

    void go_col(int tx) {
        assert (0 <= tx and tx < W);
        auto [y, x] = moves.back();
        moves.emplace_back(y, tx);
    }
};

int64_t calculate_score(const vector<pair<int, int>>& moves, const array<int64_t, H>& row, const array<int64_t, W>& col) {
    assert (not moves.empty());
    int64_t score = 0;
    REP (i, moves.size() - 1) {
        auto [ay, ax] = moves[i];
        auto [by, bx] = moves[i + 1];
        if (ay == by) {
            score += row[ay] * abs(bx - ax);
        } else if (ax == bx) {
            score += col[ax] * abs(by - ay);
        } else {
            assert (false);
        }
    }
    return score;
}

vector<pair<int, int>> solve_with_simple(int sy, int sx, int ty, int tx) {
    path_composer path(sy, sx);
    path.go_row(ty);
    path.go_col(tx);
    return *path;
}

template <class RandomEngine>
vector<pair<int, int>> solve_with_random(int sy, int sx, int ty, int tx, RandomEngine& gen) {
    assert (sy <= ty);
    assert (sx <= tx);

    path_composer path(sy, sx);
    while (true) {
        auto [y, x] = path.get_moves().back();
        if (y >= ty or x >= tx) {
            break;
        }

        int r = uniform_int_distribution<int>(0, ty - y + tx - x - 1)(gen);
        if (r < ty - y) {
            path.go_down();
        } else {
            path.go_right();
        }
    }
    path.go_row(ty);
    path.go_col(tx);
    assert (path);
    return *path;
}

vector<pair<int, int>> solve_with_scan(int sy, int sx, int ty, int tx, int scan_y) {
    assert (sy <= ty);
    assert (sx <= tx);
    assert (scan_y != sy and scan_y != ty);

    path_composer path(sy, sx);
    path.go_col(0);
    path.go_row(scan_y);
    path.go_col(W - 1);
    path.go_row(ty);
    path.go_col(tx);
    assert (path);
    return *path;
}

vector<pair<int, int>> solve_with_three(int sy, int sx, int ty, int tx, const array<int64_t, H>& row, const array<int64_t, W>& col) {
    assert (sy <= ty);
    assert (sx <= tx);

    vector<path_composer> cands;
    int64_t highscore = -1;
    auto use = [&](const path_composer& path) -> void {
        int64_t score = calculate_score(path.get_moves(), row, col);
        if (score <= highscore) {
            highscore = score;
            cands.emplace_back(path);
        }
    };

    REP3 (y1, max(0, sy - 3), min(H, ty + 3 + 1)) {
        REP3 (x, max(0, sx - 3), min(W, tx + 3 + 1)) {
            REP3 (y2, y1, min(H, ty + 3 + 1)) {
                path_composer path(sy, sx);
                path.go_row(y1);
                path.go_col(x);
                path.go_row(y2);
                path.go_col(tx);
                path.go_row(ty);
                use(path);
            }
        }
    }

    REP3 (x1, max(0, sx - 3), min(W, tx + 3 + 1)) {
        REP3 (y, max(0, sy - 3), min(H, ty + 3 + 1)) {
            REP3 (x2, x1, min(W, tx + 3 + 1)) {
                path_composer path(sy, sx);
                path.go_col(x1);
                path.go_row(y);
                path.go_col(x2);
                path.go_row(ty);
                path.go_col(tx);
                use(path);
            }
        }
    }

    while (true) {
        if (cands.empty()) {
            return solve_with_simple(sy, sx, ty, tx);
        }
        if (cands.back()) {
            break;
        }
        cands.pop_back();
    }
    return *cands.back();
}

class base_predictor {
    array<vector<pair<int, int>>, H> used_row;
    array<vector<pair<int, int>>, H> used_col;
    vector<int64_t> predicted_score;
    vector<int64_t> actual_score;

public:
    array<int64_t, H> row;
    array<int64_t, W> col;
    base_predictor() {
        fill(ALL(row), 5000);
        fill(ALL(col), 5000);
    }

    void add(const vector<pair<int, int>>& path, int64_t score) {
        assert (not path.empty());
        int j = actual_score.size();
        predicted_score.push_back(calculate_score(path, row, col));
        actual_score.push_back(score);

        map<int, int> count_row;
        map<int, int> count_col;
        REP (i, path.size() - 1) {
            auto [ay, ax] = path[i];
            auto [by, bx] = path[i + 1];
            if (ay == by) {
                count_row[ay] += 1;
            } else if (ax == bx) {
                count_col[ax] += 1;
            } else {
                assert (false);
            }
        }
        for (auto [y, k] : count_row) {
            used_row[y].emplace_back(k, j);
        }
        for (auto [x, k] : count_col) {
            used_col[x].emplace_back(k, j);
        }
    }

    template <class RandomEngine>
    void update(RandomEngine& gen, chrono::high_resolution_clock::time_point clock_end) {
        chrono::high_resolution_clock::time_point clock_begin = chrono::high_resolution_clock::now();

        int iteration = 0;
        double temprature = 1.0;
        for (; ; ++ iteration) {
            if (iteration % 128 == 0) {
                chrono::high_resolution_clock::time_point clock_now = chrono::high_resolution_clock::now();
                if (clock_now >= clock_end) {
                    break;
                }
                temprature = (clock_end - clock_now) / (clock_end - clock_begin);
            }

            int z = uniform_int_distribution<int>(0, H + W - 1)(gen);
            int64_t d = uniform_int_distribution<int>(-100, 100)(gen);

            auto& value = (z < H ? row[z] : col[z - H]);
            auto& used = (z < H ? used_row : used_col);
            if (z >= H) {
                z -= H;
            }
            if (value + d < 1000) {
                d = 1000 - value;
            } else if (9000 < value + d) {
                d = 9000 - value;
            }
            int64_t delta = 0;
            for (auto [k, j] : used[z]) {
                delta -= abs(predicted_score[j] - actual_score[j]);
                delta += abs(predicted_score[j] + k * d - actual_score[j]);
            }
            if (delta <= 0 or bernoulli_distribution(exp(- 0.001 * delta / temprature))(gen)) {
                // accept
                value += d;
                for (auto [k, j] : used[z]) {
                    predicted_score[j] += k * d;
                }
            }
        }

#ifdef VERBOSE
        int64_t loss = 0;
        REP (j, predicted_score.size()) {
            loss += abs(predicted_score[j] - actual_score[j]);
        }
        cerr << "loss = " << loss / (predicted_score.size() + 1) << endl;
#endif  // VERBOSE
    }

    void flip_hr() {
        reverse(ALL(used_row));
        reverse(ALL(row));
    }

    void flip_vr() {
        reverse(ALL(used_col));
        reverse(ALL(col));
    }
};

template <class RandomEngine>
void solve(function<tuple<int, int, int, int> ()> read, function<int64_t (const string&)> write, int K, RandomEngine& gen, chrono::high_resolution_clock::time_point clock_end) {
    chrono::high_resolution_clock::time_point clock_begin = chrono::high_resolution_clock::now();

    base_predictor predictor;

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
            predictor.flip_vr();
        };
        bool is_flipped_hr = false;
        auto flip_hr = [&]() {
            is_flipped_hr = not is_flipped_hr;
            sx = W - sx - 1;
            tx = W - tx - 1;
            REP (i, path.size()) {
                path[i].second = W - path[i].second - 1;
            }
            predictor.flip_hr();
        };
        if (sy > ty) {
            flip_vr();
        }
        if (sx > tx) {
            flip_hr();
        }

        // solve
        chrono::high_resolution_clock::time_point clock_now = chrono::high_resolution_clock::now();
        predictor.update(gen, clock_begin + (clock_end - clock_begin) * query / K);
        // path = solve_with_three(sy, sx, ty, tx, predictor.row, predictor.col);
        auto hr = make_costs_from_base_with_random(predictor.row, gen);
        auto vr = make_costs_from_base_with_random(predictor.col, gen);
        path = solve_with_dijkstra(sy, sx, ty, tx, hr, vr);

        // stop flipping
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

        // update
        history.emplace_back(path, score);
        predictor.add(path, score);
    }

#ifdef VERBOSE
    cerr << "row =";
    REP (y, H) {
        cerr << ' ' << predictor.row[y];
    }
    cerr << endl;
    cerr << "col =";
    REP (x, W) {
        cerr << ' ' << predictor.col[x];
    }
    cerr << endl;
#endif  // VERBOSE
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
