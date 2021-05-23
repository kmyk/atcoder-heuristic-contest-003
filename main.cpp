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

class base_predictor_m1 {
    vector<int64_t> actual_score;
    array<vector<pair<int, int>>, H> used_row;
    array<vector<pair<int, int>>, W> used_col;

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
        fill(ALL(cur.row), 5000);
        fill(ALL(cur.col), 5000);
        cur.loss = 0;

        best = cur;
    }

    pair<array<array<int64_t, W - 1>, H>, array<array<int64_t, H - 1>, W>> get() const {
        return get(best);
    }

    int64_t get_loss() const {
        return actual_score.empty() ? 0 : best.loss / actual_score.size();
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

        REP (i, 2) {
            auto& state = (i == 0 ? cur : best);
            auto [hr, vr] = get(state);
            state.predicted_score.push_back(calculate_score(path, hr, vr));
            state.loss += abs(state.predicted_score[j] - actual_score[j]);
        }
    }

    template <class RandomEngine>
    void update(RandomEngine& gen, chrono::high_resolution_clock::time_point clock_end) {
        chrono::high_resolution_clock::time_point clock_begin = chrono::high_resolution_clock::now();

        int iteration = 0;
        double temprature = 1.0;
        for (; ; ++ iteration) {
            if (iteration >= 1024 and iteration % 128 == 0) {
                chrono::high_resolution_clock::time_point clock_now = chrono::high_resolution_clock::now();
                if (clock_now >= clock_end) {
                    break;
                }
                temprature = (clock_end - clock_now) / (clock_end - clock_begin);
            }

            constexpr int VALUE_MIN = 1000;
            constexpr int VALUE_MAX = 9000;

            auto probability = [&](int64_t delta) -> double {
                constexpr double boltzmann = 0.0001;
                return exp(- boltzmann * delta / temprature);
            };

            bool is_row = bernoulli_distribution(0.5)(gen);
            int z = uniform_int_distribution<int>(0, H - 1)(gen);
            int64_t d = uniform_int_distribution<int>(-200, 200)(gen);

            auto& value = (is_row ? cur.row : cur.col)[z];
            if (value + d < VALUE_MIN) {
                d = VALUE_MIN - value;
            } else if (VALUE_MAX < value + d) {
                d = VALUE_MAX - value;
            }
            auto& used = (is_row ? used_row : used_col)[z];

            int64_t delta = 0;
            for (auto [j, cnt] : used) {
                delta -= abs(cur.predicted_score[j] - actual_score[j]);
                delta += abs(cur.predicted_score[j] + cnt * d - actual_score[j]);
            }

            if (delta <= 0 or bernoulli_distribution(probability(delta))(gen)) {
                // accept
                value += d;
                for (auto [j, cnt] : used) {
                    cur.loss -= abs(cur.predicted_score[j] - actual_score[j]);
                    cur.predicted_score[j] += cnt * d;
                    cur.loss += abs(cur.predicted_score[j] - actual_score[j]);
                }
            }

            if (cur.loss < best.loss) {
                best = cur;
            }
        }

#ifdef VERBOSE
        cerr << "M1: iteration = " << iteration << ", loss = " << (actual_score.empty() ? -1 : best.loss / actual_score.size()) << endl;
#endif  // VERBOSE
    }
};

class base_predictor_m2 {
    vector<int64_t> actual_score;
    array<array<vector<int>, W - 1>, H> used_hr;
    array<array<vector<int>, H - 1>, W> used_vr;

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
        fill(ALL(cur.row1), 5000);
        fill(ALL(cur.row2), 5000);
        fill(ALL(cur.col1), 5000);
        fill(ALL(cur.col2), 5000);
        cur.loss = 0;

        best = cur;
    }

    pair<array<array<int64_t, W - 1>, H>, array<array<int64_t, H - 1>, W>> get() const {
        return get(best);
    }

    int64_t get_loss() const {
        return actual_score.empty() ? 0 : best.loss / actual_score.size();
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

        REP (i, 2) {
            auto& state = (i == 0 ? cur : best);
            auto [hr, vr] = get(state);
            state.predicted_score.push_back(calculate_score(path, hr, vr));
            state.loss += abs(state.predicted_score[j] - actual_score[j]);
        }
    }

    template <class RandomEngine>
    void update(RandomEngine& gen, chrono::high_resolution_clock::time_point clock_end) {
        chrono::high_resolution_clock::time_point clock_begin = chrono::high_resolution_clock::now();

        int iteration = 0;
        double temprature = 1.0;
        for (; ; ++ iteration) {
            if (iteration >= 1024 and iteration % 128 == 0) {
                chrono::high_resolution_clock::time_point clock_now = chrono::high_resolution_clock::now();
                if (clock_now >= clock_end) {
                    break;
                }
                temprature = (clock_end - clock_now) / (clock_end - clock_begin);
            }

            constexpr int VALUE_MIN = 1000;
            constexpr int VALUE_MAX = 9000;

            auto probability = [&](int64_t delta) -> double {
                constexpr double boltzmann = 0.0001;
                return exp(- boltzmann * delta / temprature);
            };

            if (bernoulli_distribution(0.90)(gen)) {
                int i = uniform_int_distribution<int>(0, 4 - 1)(gen);
                int z = uniform_int_distribution<int>(0, H - 1)(gen);
                int64_t d = uniform_int_distribution<int>(-200, 200)(gen);

                auto& value = (i == 0 ? cur.row1 : i == 1 ? cur.row2 : i == 2 ? cur.col1 : cur.col2)[z];
                if (value + d < VALUE_MIN) {
                    d = VALUE_MIN - value;
                } else if (VALUE_MAX < value + d) {
                    d = VALUE_MAX - value;
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
                auto& used = (i < 2 ? used_hr : used_vr)[z];

                int64_t delta = 0;
                REP3 (w, l, r) {
                    for (int j : used[w]) {
                        delta -= abs(cur.predicted_score[j] - actual_score[j]);
                        delta += abs(cur.predicted_score[j] + d - actual_score[j]);
                    }
                }

                if (delta <= 0 or bernoulli_distribution(probability(delta))(gen)) {
                    // accept
                    value += d;
                    REP3 (w, l, r) {
                        for (int j : used[w]) {
                            cur.loss -= abs(cur.predicted_score[j] - actual_score[j]);
                            cur.predicted_score[j] += d;
                            cur.loss += abs(cur.predicted_score[j] - actual_score[j]);
                        }
                    }
                }

            } else {
                bool is_row = bernoulli_distribution(0.5)(gen);
                int z = uniform_int_distribution<int>(0, H - 1)(gen);
                int d = (bernoulli_distribution(0.5)(gen) ? 1 : -1);

                while (true) {
                    auto& sep = (is_row ? cur.sep_x : cur.sep_y)[z];
                    if (sep + d < 0 or sep + d > W - 1) {
                        break;
                    }
                    auto& used = (is_row ? used_hr : used_vr)[z];
                    auto& value1 = (is_row ? cur.row1 : cur.col1)[z];
                    auto& value2 = (is_row ? cur.row2 : cur.col2)[z];

                    int64_t delta = 0;
                    if (d == -1) {
                        for (int j : used[sep - 1]) {
                            delta -= abs(cur.predicted_score[j] - actual_score[j]);
                            delta += abs(cur.predicted_score[j] - value1 + value2 - actual_score[j]);
                        }
                    } else if (d == 1) {
                        for (int j : used[sep]) {
                            delta -= abs(cur.predicted_score[j] - actual_score[j]);
                            delta += abs(cur.predicted_score[j] - value2 + value1 - actual_score[j]);
                        }
                    } else {
                        assert (false);
                    }

                    if (delta <= 0 or bernoulli_distribution(probability(delta))(gen)) {
                        // accept
                        if (d == -1) {
                            for (int j : used[sep - 1]) {
                                cur.loss -= abs(cur.predicted_score[j] - actual_score[j]);
                                cur.predicted_score[j] += - value1 + value2;
                                cur.loss += abs(cur.predicted_score[j] - actual_score[j]);
                            }
                            sep -= 1;
                        } else if (d == 1) {
                            for (int j : used[sep]) {
                                cur.loss -= abs(cur.predicted_score[j] - actual_score[j]);
                                cur.predicted_score[j] += - value2 + value1;
                                cur.loss += abs(cur.predicted_score[j] - actual_score[j]);
                            }
                            sep += 1;
                        } else {
                            assert (false);
                        }
                        assert (0 <= sep and sep <= H - 1);

                    } else {
                        // reject
                        break;
                    }
                }
            }

            if (cur.loss < best.loss) {
                best = cur;
            }
        }

#ifdef VERBOSE
        cerr << "M2: iteration = " << iteration << ", loss = " << (actual_score.empty() ? -1 : best.loss / actual_score.size()) << endl;
#endif  // VERBOSE
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
        cerr << "use M" << (is_m1 ? 1 : 2) << ": (" << sy << ", " << sx << ") -> (" << ty << ", " << tx << "): " << score << " (M1 " << calculate_score(path, hr1, vr1) - score << ", M2 " << calculate_score(path, hr2, vr2) - score<< ")" << endl;
#endif  // VERBOSE
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
