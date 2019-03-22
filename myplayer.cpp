#include <algorithm>
#include <iostream>
#include <queue>
#include <map>
#include <unistd.h>
#include <cassert>
#include <fstream>
#include <memory>
#include <thread>
#include <mutex>
#include <cmath>
#include "raceInfo.hpp"

#define LOCAL

#ifdef LOCAL
ofstream fout("output.txt");
#endif

class XorShift {
public:
    unsigned long x, y, z, w;
    XorShift() {
        x = 123456789; y = 362436069; z = 521288629; w = 88675123;
    }
    XorShift(unsigned long seed) {
        XorShift();
        w = seed;
        for (int i = 0; i < 100; ++i) (*this)();
    }
    unsigned long operator()() {
        unsigned long t = x ^ (x << 11);
        x = y; y = z; z = w;
        return w = (w ^ (w >> 19)) ^ (t ^ (t >> 8));
    }
};

XorShift rnd((unsigned long)(getuid()));

const int INF = 1e8;

struct State {
    int step;
    PlayerState player;
    Acceleration action;
    shared_ptr<State> parent;
    bool is_goal;
    double goal_time;
    double eval;
    
    State() { step = -2; parent = nullptr; is_goal = false; goal_time = eval = -INF; }
    State(int step, PlayerState ps, Acceleration a, shared_ptr<State> p) :step(step), player(ps), action(a), parent(p) {
        is_goal = false;
        goal_time = eval = -INF;
    }
    State(shared_ptr<State> s) {
        parent = s;
        step = s->step + 1;
        player = s->player;
        is_goal = s->is_goal;
        eval = s->eval;
    }
    
    bool operator<(const State& s) const {
        if (is_goal and s.is_goal) {
            return INF + -goal_time < INF + -s.goal_time;
        }
        else if (is_goal) return false;
        else if (s.is_goal) return true;
        else return eval < s.eval;
    }
    
    bool operator>(const State &s) const {
        return eval > s.eval;
        if (is_goal and s.is_goal) {
            return INF + -goal_time > INF + -s.goal_time;
        }
        else if (is_goal) return true;
        else if (s.is_goal) return false;
        else return eval > s.eval;
    }
};

// DOWN, RIGHT, UP, LEFT
int dr[] = { 1, 0, -1, 0, 1, 1, -1, -1 };
int dc[] = { 0, 1, 0, -1, 1, -1, 1, -1 };
Position DIR[8];

const int MAXV = 3000;
struct edge {
    int to, cost;
    edge() {}
    edge(int t, int c) :to(t), cost(c) {}
};
int V = MAXV;
vector<edge> G[MAXV];
int dist[MAXV];

void dijkstra(int s) {
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> que;
    fill(dist, dist+V, INF);
    dist[s] = 0;
    que.push({0, s});
    while (que.size()) {
        pair<int, int> p = que.top(); que.pop();
        int v = p.second;
        if (dist[v] < p.first) continue;
        for (edge& e : G[v]) {
            if (dist[e.to] > dist[v] + e.cost) {
                dist[e.to] = dist[v] + e.cost;
                que.push({dist[e.to], e.to});
            }
        }
    }
}

class Solver {
private:
    int LENGTH, WIDTH, VISION;
    RaceInfo info;
    inline bool in_width(Position p) {
        return 0 <= p.x and p.x < WIDTH;
    }
    inline bool in_length(Position p) {
        return 0 <= p.y and p.y < LENGTH;
    }
    
    void make_distance();
public:
    void initialize();
    void loop();
    void solve();
    Acceleration think(PlayerState player);
    void run();
};

void Solver::initialize() {
    cin >> course;
    cout << "0" << endl;
    cout.flush();
    
    for (int d=0; d<8; ++d) {
        DIR[d] = Position(dc[d], dr[d]);
    }
    WIDTH = course.width;
    LENGTH = course.length;
    VISION = course.vision;
}

void Solver::loop() {
    while (!cin.eof()) {
        cin >> info;
        solve();
        cout.flush();
        while (isspace(cin.peek())) cin.ignore(1);
    }
}

void Solver::run() {
    initialize();
    loop();
}

void Solver::make_distance() {
    // 何回で一番上まで行けるか
    for (int i=0; i<MAXV; ++i) {
        G[i].clear();
    }
    int row = LENGTH-1;
    for (int r=0; r<LENGTH; ++r) {
        if (info.squares[r][0] == -1) {
            row = r-1;
            break;
        }
    }
    for (int r=row; r>=0; --r) {
        for (int c=0; c<WIDTH; ++c) {
            int idx = r*WIDTH+c;
            
            // 壁なので無視
            if (info.squares[r][c] == 1) continue;
            
            // 一番上の行ならば
            if (r == row) {
                G[MAXV-1].emplace_back(idx, 1);
                G[idx].emplace_back(MAXV-1, 1);
            }
            
            // 4近傍
            bool dir[4];
            for (int d=0; d<4; ++d) {
                int nr = r+dr[d], nc = c+dc[d];
                if (0 <= nr and nr < LENGTH and 0 <= nc and nc < WIDTH and info.squares[nr][nc] != 1) {
                    int idx2 = nr*WIDTH+nc;
                    G[idx].emplace_back(idx2, 1);
                    dir[d] = true;
                }
                else dir[d] = false;
            }
            
            if (dir[0] and dir[3] and info.squares[r+1][c-1] != 1) {
                G[idx].emplace_back((r+1)*WIDTH+(c-1), 1);
            }
            if (dir[0] and dir[1] and info.squares[r+1][c+1] != 1) {
                G[idx].emplace_back((r+1)*WIDTH+(c+1), 1);
            }
            if (dir[1] and dir[2] and info.squares[r-1][c+1] != 1) {
                G[idx].emplace_back((r-1)*WIDTH+(c+1), 1);
            }
            if (dir[2] and dir[3] and info.squares[r-1][c-1] != 1) {
                G[idx].emplace_back((r-1)*WIDTH+(c-1), 1);
            }
            
        }
    }
    dijkstra(MAXV-1);
    for (int i=0; i<MAXV; ++i) {
        dist[i] *= -1;
    }
}

Acceleration Solver::think(PlayerState player) {
    PlayerState me(player.position, player.velocity);
    State first_state(0, me, Acceleration(0, 0), nullptr);
    
    // beam search
    priority_queue<State> now;
    now.push(first_state);
    
    // ビーム幅
    int k = 300, MAX_TURN = 10;
    for (int t=0; t<MAX_TURN; ++t) {
        priority_queue<State> next;
        for (int i=0; i<k; ++i) {
            if (now.empty()) break;
            State now_state = now.top(); now.pop();
            
            // ゴールしていたら，次に持ち越す
            if (now_state.is_goal) {
                next.push(now_state);
                continue;
            }
            
            // ここで見る
            for (int ax = -1; ax <= 1; ++ax) {
                for (int ay = -1; ay <= 1; ++ay) {
                    
                    // 次のコマンド
                    Acceleration accel(ax, ay);
                    
                    Velocity nextv = now_state.player.velocity + accel;
                    Position nextpos = now_state.player.position + nextv;
                    
                    if (!in_width(nextpos)) continue;
                    if (in_length(nextpos) and info.squares[nextpos.y][nextpos.x] == 1) continue;
                    
                    Movement nextmove(now_state.player.position, nextpos);
                    list<Position> touched = nextmove.touchedSquares();
                    
                    bool ok = true, goal = false;
                    for (Position& p : touched) {
                        if (!in_width(p) or p.y < 0) {
                            ok = false;
                            break;
                        }
                        if (p.y >= LENGTH) {
                            goal = true;
                        }
                        if (in_length(p) and info.squares[p.y][p.x] == 1) {
                            ok = false;
                            break;
                        }
                    }
                    
                    if (ok) {
                        State next_state(make_shared<State>(now_state));
                        if (!goal) {
                            next_state.player.position = nextpos;
                            next_state.action = accel;
                            if (info.squares[nextpos.y][nextpos.x] == 2) {
                                nextv = Velocity(0, 0);
                                next_state.player.velocity = Velocity(0, 0);
                            }
                            else {
                                next_state.player.velocity = nextv;
                            }
                            next_state.eval += -now_state.step + pow(dist[nextpos.y*WIDTH+nextpos.x], 3);
                            //next_state.eval += (info.squares[nextpos.y][nextpos.x] == 0 ? 10 : 0);
                            next.push(next_state);
                        }
                        else {
                            next_state.is_goal = true;
                            next_state.action = accel;
                            next_state.goal_time = next_state.step + (LENGTH - next_state.player.position.y - 0.5) / next_state.player.velocity.y;
                            next.push(next_state);
                        }
                    }
                }
            }
        }
        now = next;
    }
    
    if (now.empty()) {
        return Acceleration(0, 0);
    }
    else {
        vector<State> kouho;
        double best_score = now.top().eval;
        State best_state = now.top();
        kouho.push_back(best_state);
        now.pop();
        while (now.size()) {
            if (now.top().eval == best_score) {
                kouho.push_back(now.top());
            }
            else break;
            now.pop();
        }
        best_state = kouho[rnd()%kouho.size()];
        while (best_state.step > 1) {
            best_state = *best_state.parent;
        }
        return Acceleration(best_state.action.x, best_state.action.y);
    }
}

void Solver::solve() {
    // ループ内
    make_distance();
    
    Acceleration enemy_accel = think(PlayerState(info.opponent.position, info.opponent.velocity));
    Velocity enemy_v = info.opponent.velocity + enemy_accel;
    Position enemy_p = info.opponent.position + enemy_v;
    Movement enemy_estimate_move(info.opponent.position, enemy_p);
    list<Position> enemy_touched = enemy_estimate_move.touchedSquares();
    
    PlayerState me(info.me.position, info.me.velocity);
    State first_state(0, me, Acceleration(0, 0), nullptr);
    
    // beam search
    priority_queue<State> now;
    now.push(first_state);
    
    // ビーム幅
    int k = 300, MAX_TURN = 10;
    for (int t=0; t<MAX_TURN; ++t) {
        priority_queue<State> next;
        for (int i=0; i<k; ++i) {
            if (now.empty()) break;
            State now_state = now.top(); now.pop();
            
            // ゴールしていたら，次に持ち越す
            if (now_state.is_goal) {
                next.push(now_state);
                continue;
            }
            
            // ここで見る
            bool dame = false;
            for (int ax = -1; ax <= 1; ++ax) {
                for (int ay = -1; ay <= 1; ++ay) {
                    
                    // 次のコマンド
                    Acceleration accel(ax, ay);
                    
                    Velocity nextv = now_state.player.velocity + accel;
                    Position nextpos = now_state.player.position + nextv;
                    
                    if (!in_width(nextpos)) continue;
                    if (in_length(nextpos) and info.squares[nextpos.y][nextpos.x] == 1) continue;
                    
                    Movement nextmove(now_state.player.position, nextpos);
                    list<Position> touched = nextmove.touchedSquares();
                    
                    bool ok = true, goal = false, crash = false;
                    for (Position& p : touched) {
                        if (!in_width(p) or p.y < 0) {
                            ok = false;
                            break;
                        }
                        if (p.y >= LENGTH) {
                            goal = true;
                        }
                        if (in_length(p) and info.squares[p.y][p.x] == 1) {
                            ok = false;
                            break;
                        }
                        
                        if (t == 0) {
                            for (Position& p2 : enemy_touched) {
                                if (p == p2) {
                                    // 衝突します
                                    crash = true;
                                }
                            }
                        }
                    }
                    
                    
                    if (ok) {
                        State next_state(make_shared<State>(now_state));
                        
                        if (crash) {
                            if (info.me.position.y == info.opponent.position.y) {
                                if (info.me.position.x == info.opponent.position.x) ;
                                else {
                                    next_state.eval += (info.me.position.x < info.opponent.position.x ? 1 : -1) * 100;
                                }
                            }
                            else {
                                next_state.eval += (info.me.position.y < info.opponent.position.y ? 1 : -1) * 100;
                            }
                        }
                        
                        if (!goal) {
                            next_state.player.position = nextpos;
                            next_state.action = accel;
                            if (info.squares[nextpos.y][nextpos.x] == 2) {
                                nextv = Velocity(0, 0);
                                next_state.player.velocity = Velocity(0, 0);
                            }
                            else {
                                next_state.player.velocity = nextv;
                            }
                            next_state.eval += -now_state.step + pow(dist[nextpos.y*WIDTH+nextpos.x], 3);
                            //next_state.eval += (info.squares[nextpos.y][nextpos.x] == 0 ? 10 : 0);
                            next.push(next_state);
                        }
                        else {
                            next_state.is_goal = true;
                            next_state.action = accel;
                            next_state.goal_time = next_state.step + (LENGTH - next_state.player.position.y - 0.5) / next_state.player.velocity.y;
                            next.push(next_state);
                        }
                    }
                    else {
                        if (!dame and now_state.player.position.y > 0) {
                            State next_state(make_shared<State>(now_state));
                            dame = true;
                            next_state.player.position = now_state.player.position;
                            next_state.action = accel;
                            next_state.player.velocity = Velocity(0, 0);
                            int idx = now_state.player.position.y * WIDTH + now_state.player.position.x;
                            next_state.eval += -now_state.step + pow(dist[now_state.player.position.y * WIDTH + now_state.player.position.x], 3);
                            next.push(next_state);
                        }
                    }
                }
            }
        }
        now = next;
    }
    
    if (now.empty()) {
        cout << "0 0" << endl;
    }
    else {
        vector<State> kouho;
        double best_score = now.top().eval;
        State best_state = now.top();
        kouho.push_back(best_state);
        now.pop();
        while (now.size()) {
            if (now.top().eval == best_score) {
                kouho.push_back(now.top());
            }
            else break;
            now.pop();
        }
        best_state = kouho[rnd()%kouho.size()];
        while (best_state.step > 1) {
            best_state = *best_state.parent;
        }
        cout << best_state.action.x << ' ' << best_state.action.y << endl;
    }
}

int main() {
    Solver solver;
    solver.run();
}
