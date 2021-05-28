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

constexpr int VALUE_MIN = 1000;
constexpr int VALUE_MAX = 9000;
constexpr int VALUE_CENTER = (VALUE_MAX + VALUE_MIN) / 2;
constexpr int DELTA = 2000;

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
    assert (path.front() == make_pair(sy, sx));
    assert (path.back() == make_pair(ty, tx));
    assert ((set<pair<int, int>>(ALL(path)).size() == path.size()));
    return path;
}

int64_t calculate_score(const vector<pair<int, int>>& path, const array<array<int64_t, W - 1>, H>& hr, const array<array<int64_t, H - 1>, W>& vr) {
    assert (not path.empty());
    int64_t score = 0;
    REP (i, path.size() - 1) {
        auto [ay, ax] = path[i];
        auto [by, bx] = path[i + 1];
        if (ay == by and abs(bx - ax) == 1) {
            score += hr[ay][min(ax, bx)];
        } else if (abs(by - ay) == 1 and ax == bx) {
            score += vr[ax][min(ay, by)];
        } else {
            assert (false);
        }
    }
    return score;
}

struct row_col_history {
    vector<int64_t> actual_score;
    array<vector<pair<int, int>>, H> used_row;
    array<vector<pair<int, int>>, W> used_col;

    int size() const {
        return actual_score.size();
    }

    void add(const vector<pair<int, int>>& path, int64_t score) {
        assert (not path.empty());
        int j = actual_score.size();
        actual_score.push_back(score);

        auto use = [&](vector<pair<int, int>>& used) {
            if (used.empty() or used.back().first != j) {
                used.emplace_back(j, 0);
            }
            used.back().second += 1;
        };
        REP (i, path.size() - 1) {
            auto [ay, ax] = path[i];
            auto [by, bx] = path[i + 1];
            if (ay == by) {
                use(used_row[ay]);
            } else if (ax == bx) {
                use(used_col[ax]);
            } else {
                assert (false);
            }
        }
    }

};

class base_predictor_m1 {
    row_col_history history;

    struct prediction_state {
        array<int64_t, H> row;
        array<int64_t, W> col;
        vector<int64_t> predicted_score;
        int64_t loss;
    };

    prediction_state cur;
    prediction_state best;

    pair<array<array<int64_t, W - 1>, H>, array<array<int64_t, H - 1>, W>> get(const prediction_state& state) const {
        array<array<int64_t, W - 1>, H> hr;
        array<array<int64_t, H - 1>, W> vr;
        REP (y, H) {
            fill(ALL(hr[y]), state.row[y]);
        }
        REP (x, W) {
            fill(ALL(vr[x]), state.col[x]);
        }
        return {hr, vr};
    }

public:
    base_predictor_m1() {
        fill(ALL(cur.row), VALUE_CENTER);
        fill(ALL(cur.col), VALUE_CENTER);
        cur.loss = 0;

        best = cur;
    }

    pair<array<array<int64_t, W - 1>, H>, array<array<int64_t, H - 1>, W>> get() const {
        return get(best);
    }

    int64_t get_loss() const {
        return history.size() == 0 ? 0 : best.loss / history.size();
    }

    void add(const vector<pair<int, int>>& path, int64_t score) {
        history.add(path, score);

        REP (i, 2) {
            auto& state = (i == 0 ? cur : best);
            auto [hr, vr] = get(state);
            int64_t predicted_score = calculate_score(path, hr, vr);
            state.predicted_score.push_back(predicted_score);
            state.loss += abs(predicted_score - score);
        }
    }

    template <class RandomEngine>
    void update(RandomEngine& gen, chrono::high_resolution_clock::time_point clock_end) {
        chrono::high_resolution_clock::time_point clock_begin = chrono::high_resolution_clock::now();

        int iteration = 0;
        double temprature = 1.0;
        for (; ; ++ iteration) {
            if (iteration % 32 == 0) {
                chrono::high_resolution_clock::time_point clock_now = chrono::high_resolution_clock::now();
                if (clock_now >= clock_end) {
                    break;
                }
                temprature = (clock_end - clock_now) / (clock_end - clock_begin);
            }

            auto probability = [&](int64_t delta) -> double {
                constexpr double boltzmann = 0.0001;
                return exp(- boltzmann * delta / temprature);
            };

            auto try_update_row = [&](bool is_row, int z, int64_t d) -> bool {
                auto& value = (is_row ? cur.row : cur.col)[z];
                if (value + d < VALUE_MIN) {
                    d = VALUE_MIN - value;
                } else if (VALUE_MAX < value + d) {
                    d = VALUE_MAX - value;
                }
                if (d == 0) {
                    return false;
                }
                auto& used = (is_row ? history.used_row : history.used_col)[z];

                int64_t delta = 0;
                for (auto [j, cnt] : used) {
                    delta -= abs(cur.predicted_score[j] - history.actual_score[j]);
                    delta += abs(cur.predicted_score[j] + cnt * d - history.actual_score[j]);
                }

                if (delta <= 0 or bernoulli_distribution(probability(delta))(gen)) {
                    // accept
                    cur.loss += delta;
                    value += d;
                    for (auto [j, cnt] : used) {
                        cur.predicted_score[j] += cnt * d;
                    }
                    if (cur.loss < best.loss) {
                        best = cur;
                    }
                    return true;

                } else {
                    return false;
                }
            };

            bool is_row = bernoulli_distribution(0.5)(gen);
            int z = uniform_int_distribution<int>(0, H - 1)(gen);
            int64_t d = uniform_int_distribution<int>(-200, 200)(gen);
            try_update_row(is_row, z, d);
        }

#ifdef VERBOSE
        cerr << "M1: iteration = " << iteration << ", loss = " << (history.size() == 0 ? -1 : best.loss / history.size()) << endl;
#endif  // VERBOSE
    }

    void report() const {
        cerr << "M1" << endl;
        cerr << "    loss = " << get_loss() << endl;
        cerr << "    row =";
        REP (y, H) {
            cerr << ' ' << best.row[y];
        }
        cerr << endl;
        cerr << "    col =";
        REP (x, W) {
            cerr << ' ' << best.col[x];
        }
        cerr << endl;
    }
};

struct hr_vr_history {
    vector<int64_t> actual_score;
    array<array<vector<int>, W - 1>, H> used_hr;
    array<array<vector<int>, H - 1>, W> used_vr;

    int size() const {
        return actual_score.size();
    }

    void add(const vector<pair<int, int>>& path, int64_t score) {
        assert (not path.empty());
        int j = actual_score.size();
        actual_score.push_back(score);

        REP (i, path.size() - 1) {
            auto [ay, ax] = path[i];
            auto [by, bx] = path[i + 1];
            if (ay == by) {
                used_hr[ay][min(ax, bx)].push_back(j);
            } else if (ax == bx) {
                used_vr[ax][min(ay, by)].push_back(j);
            } else {
                assert (false);
            }
        }
    }
};

class base_predictor_m2 {
    hr_vr_history history;

    struct prediction_state {
        array<int, H> sep_x;
        array<int, W> sep_y;
        array<int64_t, H> row1, row2;
        array<int64_t, W> col1, col2;
        vector<int64_t> predicted_score;
        int64_t loss;
    };

    prediction_state cur;
    prediction_state best;

    pair<array<array<int64_t, W - 1>, H>, array<array<int64_t, H - 1>, W>> get(const prediction_state& state) const {
        array<array<int64_t, W - 1>, H> hr;
        array<array<int64_t, H - 1>, W> vr;
        REP (y, H) {
            REP (x, W - 1) {
                hr[y][x] = (x < state.sep_x[y] ? state.row1 : state.row2)[y];
            }
        }
        REP (x, W) {
            REP (y, H - 1) {
                vr[x][y] = (y < state.sep_y[x] ? state.col1 : state.col2)[x];
            }
        }
        return {hr, vr};
    }

public:
    base_predictor_m2() {
        fill(ALL(cur.sep_x), W / 2);
        fill(ALL(cur.sep_y), H / 2);
        fill(ALL(cur.row1), VALUE_CENTER);
        fill(ALL(cur.row2), VALUE_CENTER);
        fill(ALL(cur.col1), VALUE_CENTER);
        fill(ALL(cur.col2), VALUE_CENTER);
        cur.loss = 0;

        best = cur;
    }

    pair<array<array<int64_t, W - 1>, H>, array<array<int64_t, H - 1>, W>> get() const {
        return get(best);
    }

    int64_t get_loss() const {
        return history.size() == 0 ? 0 : best.loss / history.size();
    }

    void add(const vector<pair<int, int>>& path, int64_t score) {
        history.add(path, score);

        REP (i, 2) {
            auto& state = (i == 0 ? cur : best);
            auto [hr, vr] = get(state);
            int64_t predicted_score = calculate_score(path, hr, vr);
            state.predicted_score.push_back(predicted_score);
            state.loss += abs(predicted_score - score);
        }
    }

    template <class RandomEngine>
    void update(RandomEngine& gen, chrono::high_resolution_clock::time_point clock_end) {
        chrono::high_resolution_clock::time_point clock_begin = chrono::high_resolution_clock::now();

        int iteration = 0;
        double temprature = 1.0;
        for (; ; ++ iteration) {
            if (iteration % 32 == 0) {
                chrono::high_resolution_clock::time_point clock_now = chrono::high_resolution_clock::now();
                if (clock_now >= clock_end) {
                    break;
                }
                temprature = (clock_end - clock_now) / (clock_end - clock_begin);
            }

            auto probability = [&](int64_t delta) -> double {
                constexpr double boltzmann = 0.0001;
                return exp(- boltzmann * delta / temprature);
            };

            auto try_update_row = [&](int i, int z, int64_t d) -> bool {
                assert (0 <= i and i < 4);

                auto& value = (i == 0 ? cur.row1 : i == 1 ? cur.row2 : i == 2 ? cur.col1 : cur.col2)[z];
                if (value + d < VALUE_MIN) {
                    d = VALUE_MIN - value;
                } else if (VALUE_MAX < value + d) {
                    d = VALUE_MAX - value;
                }
                if (d == 0) {
                    return false;
                }

                int l = 0;
                int r = H - 1;
                if (i == 0) {
                    r = cur.sep_x[z];
                } else if (i == 1) {
                    l = cur.sep_x[z];
                } else if (i == 2) {
                    r = cur.sep_y[z];
                } else if (i == 3) {
                    l = cur.sep_y[z];
                } else {
                    assert (false);
                }
                auto& used = (i < 2 ? history.used_hr : history.used_vr)[z];

                int64_t delta = 0;
                REP3 (w, l, r) {
                    for (int j : used[w]) {
                        delta -= abs(cur.predicted_score[j] - history.actual_score[j]);
                        delta += abs(cur.predicted_score[j] + d - history.actual_score[j]);
                    }
                }

                if (delta <= 0 or bernoulli_distribution(probability(delta))(gen)) {
                    // accept
                    cur.loss += delta;
                    value += d;
                    REP3 (w, l, r) {
                        for (int j : used[w]) {
                            cur.predicted_score[j] += d;
                        }
                    }
                    if (cur.loss < best.loss) {
                        best = cur;
                    }
                    return true;

                } else {
                    // reject
                    return false;
                }
            };

            auto try_update_sep = [&](bool is_row, int z, int nsep) -> bool {
                assert (1 <= nsep and nsep <= H - 2);

                auto& sep = (is_row ? cur.sep_x : cur.sep_y)[z];
                auto& used = (is_row ? history.used_hr : history.used_vr)[z];
                auto& value1 = (is_row ? cur.row1 : cur.col1)[z];
                auto& value2 = (is_row ? cur.row2 : cur.col2)[z];

                if (nsep == sep) {
                    return false;
                }

                int64_t delta = 0;
                REP3 (k, min(sep, nsep), max(sep, nsep)) {
                    for (int j : used[k]) {
                        delta -= abs(cur.predicted_score[j] - history.actual_score[j]);
                        cur.predicted_score[j] += (sep < nsep ? - value2 + value1 : - value1 + value2);
                        delta += abs(cur.predicted_score[j] - history.actual_score[j]);
                    }
                }

                if (delta <= 0 or bernoulli_distribution(probability(delta))(gen)) {
                    // accept
                    cur.loss += delta;
                    sep = nsep;
                    if (cur.loss < best.loss) {
                        best = cur;
                    }
                    return true;

                } else {
                    // reject
                    REP3 (k, min(sep, nsep), max(sep, nsep)) {
                        for (int j : used[k]) {
                            cur.predicted_score[j] -= (sep < nsep ? - value2 + value1 : - value1 + value2);
                        }
                    }
                    return false;
                }
            };

            if (bernoulli_distribution(0.9)(gen)) {
                int i = uniform_int_distribution(0, 4 - 1)(gen);
                int z = uniform_int_distribution<int>(0, H - 1)(gen);
                int64_t d = uniform_int_distribution(-200, 200)(gen);

                try_update_row(i, z, d);

            } else {
                bool is_row = bernoulli_distribution(0.5)(gen);
                int z = uniform_int_distribution<int>(0, H - 1)(gen);
                int d = (bernoulli_distribution(0.5)(gen) ? 1 : -1);
                int sep = (is_row ? cur.sep_x : cur.sep_y)[z];
                int nsep = sep + d;

                if (1 <= nsep and nsep <= W - 2) {
                    try_update_sep(is_row, z, nsep);
                }
            }
        }

#ifdef VERBOSE
        cerr << "M2: iteration = " << iteration << ", loss = " << (history.size() == 0 ? -1 : best.loss / history.size()) << endl;
#endif  // VERBOSE
    }

    void report() const {
        cerr << "M2" << endl;
        cerr << "    loss = " << get_loss() << endl;
        cerr << "    row1 =";
        REP (y, H) {
            cerr << ' ' << best.row1[y];
        }
        cerr << endl;
        cerr << "    row2 =";
        REP (y, H) {
            cerr << ' ' << best.row2[y];
        }
        cerr << endl;
        cerr << "    sep_x =";
        REP (y, H) {
            cerr << ' ' << best.sep_x[y];
        }
        cerr << endl;
        cerr << "    col1 =";
        REP (x, W) {
            cerr << ' ' << best.col1[x];
        }
        cerr << endl;
        cerr << "    col2 =";
        REP (x, W) {
            cerr << ' ' << best.col2[x];
        }
        cerr << endl;
        cerr << "    sep_y =";
        REP (x, W) {
            cerr << ' ' << best.sep_y[x];
        }
        cerr << endl;
    }
};

template <class RandomEngine>
void solve(function<tuple<int, int, int, int> ()> read, function<int64_t (const string&)> write, int K, RandomEngine& gen, chrono::high_resolution_clock::time_point clock_end) {
    chrono::high_resolution_clock::time_point clock_begin = chrono::high_resolution_clock::now();

    base_predictor_m1 predictor1;
    base_predictor_m2 predictor2;

    vector<pair<vector<pair<int, int>>, int64_t>> history;
    REP (query, K) {
        // input
        int sy, sx, ty, tx;
        tie(sy, sx, ty, tx) = read();

        // solve
        bool is_m1 = (predictor1.get_loss() < predictor2.get_loss() + 1000);
        auto [hr, vr] = (is_m1 ? predictor1.get() : predictor2.get());
        vector<pair<int, int>> path = solve_with_dijkstra(sy, sx, ty, tx, hr, vr);

        // output
        int64_t score = write(get_command_from_path(path));
        history.emplace_back(path, score);

        // update
        predictor1.add(path, score);
        predictor2.add(path, score);
        predictor1.update(gen, clock_begin + (clock_end - clock_begin) * (2 * query + 1) / (2 * K));
        predictor2.update(gen, clock_begin + (clock_end - clock_begin) * (2 * query + 2) / (2 * K));

#ifdef VERBOSE
        auto [hr1, vr1] = predictor1.get();
        auto [hr2, vr2] = predictor2.get();
        cerr << "use M" << (is_m1 ? 1 : 2) << " (M1 " << (calculate_score(path, hr1, vr1) - score) / static_cast<int>(path.size()) << ", M2 " << (calculate_score(path, hr2, vr2) - score) / static_cast<int>(path.size()) << ")" << endl;
#endif  // VERBOSE
    }
#ifdef VERBOSE
    predictor1.report();
    predictor2.report();
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
