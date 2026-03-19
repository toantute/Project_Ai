#include <iostream>
#include <queue>
#include <vector>
#include <utility>
#include <stack>
#include <math.h>
using std::cin;
using std::cout;
using std::queue;
using std::priority_queue;
using std::pair;
using std::endl; 
using std::string;
using std::stack;
using std::vector;
double wavg; 
typedef struct state{
    int x, y, g, d;
    double h;
    int px,py;
}state;
typedef struct cmp1{ 
    bool operator()(const state &a, const state &b){
        return a.g > b.g;
    }
}cmp1;
typedef struct cmp2{ 
    bool operator()(const state &a, const state &b){
        return a.h > b.h;
    }
}cmp2;
typedef struct cmp3{ 
    bool operator()(const state &a, const state &b){
        return (a.g + a.h) > (b.g+b.h);
    }
}cmp3;
void arrInit(void ***a, int m, int n, int type){
    if (!type){
        int **p;
        p = (int**)malloc(sizeof(int*)*m);
        for (int i = 0 ; i < m ; i ++) p[i] = (int*)malloc(sizeof(int)*n);
        (*a) = (void**)p;
    }
    else if (type==1){
        bool **p;
        p = (bool**)malloc(sizeof(bool*)*m);
        for (int i = 0 ; i < m ; i ++) p[i] = (bool*)malloc(sizeof(bool)*n);
        (*a) = (void**)p;
    }
    else {
        pair<int,int> **p;
        p = (pair<int,int>**)malloc(sizeof(pair<int,int>*)*m);
        for (int i = 0 ; i < m ; i ++) p[i] = (pair<int,int>*)malloc(sizeof(pair<int,int>)*n);
        (*a) = (void**)p;
    }
}
void arrFree(void ***a, int m, int type){
    if (!type){
        int **p = (int**)(*a);
        for (int i = 0 ; i < m ; i ++) free(p[i]); 
        free(p);
    }
    else if (type==1){
        bool **p = (bool**)(*a);
        for (int i = 0 ; i < m ; i ++) free(p[i]); 
        free(p);
    }
    else {
        pair<int,int> **p = (pair<int,int>**)(*a);
        for (int i = 0 ; i < m ; i ++) free(p[i]); 
        free(p);
    }
    (*a) = NULL; 
}
int dx[] = {-1,0,1,0};
int dy[] = {0,1,0,-1};
void trace(pair<int,int> **parent, int x, int y){
    if (parent[x][y].first != x || parent[x][y].second != y) 
    trace(parent,parent[x][y].first,parent[x][y].second);
    cout << "("<< x << "," << y << ")" << " ";
}
bool valid1(int **M,bool **visited, int m, int n, int x, int y){
    return x>=0&&y>=0&&x<m&&y<n&&M[x][y]!=0&&!visited[x][y]; 
}
bool valid2(int **M, int m, int n, int x, int y){
    return x>=0&&y>=0&&x<m&&y<n&&M[x][y]!=0;
}
// 0 = manhattan, 1 = euclidean
double h(int x, int y, int tx, int ty, int type){
    int disx = x > tx ? x-tx : tx - x;
    int disy = y > ty ? y-ty : ty - y;
    if (!type) return (disx + disy) * wavg;
    else return sqrt(disx*disx+disy*disy) * wavg;
}
bool BFS(int** M, bool **visited, pair<int,int> **parent, int w[], int m ,int n,
int sx, int sy, int tx, int ty){
    queue<state> fringe;
    state cur = {sx,sy,0};
    fringe.push(cur);
    visited[sx][sy] = true; 
    parent[sx][sy] = {sx,sy};
    while (!fringe.empty()){
        cur = fringe.front(); 
        fringe.pop();
        if (cur.x == tx && cur.y == ty) { 
            cout << cur.g << endl;
            return true; 
        }
        for (int i = 0 ; i < 4 ; i++){
            int nx = cur.x + dx[i];
            int ny = cur.y + dy[i];
            if (valid1(M,visited,m,n,nx,ny)) {
                state next = {nx,ny,cur.g+w[M[nx][ny]]};
                parent[nx][ny] = {cur.x,cur.y};
                visited[nx][ny] = true;
                fringe.push(next);
            }
        }
    }
    return false;
}
bool DFS(int** M, bool **visited, pair<int,int> **parent, int w[], int m ,int n,
int sx, int sy, int tx, int ty){
    stack<state> fringe;
    state cur = {sx,sy,0};
    fringe.push(cur);
    visited[sx][sy] = true; 
    parent[sx][sy] = {sx,sy};
    while (!fringe.empty()){
        cur = fringe.top(); 
        fringe.pop();
        if (cur.x == tx && cur.y == ty) { 
            cout << cur.g << endl;
            return true; 
        }
        for (int i = 0 ; i < 4 ; i++){
            int nx = cur.x + dx[i];
            int ny = cur.y + dy[i];
            if (valid1(M,visited,m,n,nx,ny)) {
                state next = {nx,ny,cur.g+w[M[nx][ny]]};
                parent[nx][ny] = {cur.x,cur.y};
                visited[nx][ny] = true;
                fringe.push(next);
            }
        }
    }
    return false;
}
bool UCS(int** M, pair<int,int> **parent, int w[], int m ,int n,
int sx, int sy, int tx, int ty){
    int dist[m][n]; 
    for (int i = 0 ; i < m ; i++){
        for (int j = 0 ; j < n ; j++){
            dist[i][j] = INT_MAX; 
        }
    }
    dist[sx][sy] = 0;
    priority_queue<state,vector<state>,cmp1> fringe; 
    state cur = {sx,sy,0};
    fringe.push(cur);
    parent[sx][sy] = {sx,sy};
    while (!fringe.empty()){
        cur = fringe.top(); 
        fringe.pop();
        if (cur.g > dist[cur.x][cur.y]) continue;
        if (cur.x == tx && cur.y == ty) { 
            cout << cur.g << endl;
            return true; 
        }
        for (int i = 0 ; i < 4 ; i++){
            int nx = cur.x + dx[i];
            int ny = cur.y + dy[i];
            if (valid2(M,m,n,nx,ny)) {
                state next = {nx,ny,cur.g+w[M[nx][ny]]};
                if (dist[nx][ny]==INT_MAX||next.g<dist[nx][ny]) {
                    parent[nx][ny] = {cur.x,cur.y};
                    dist[nx][ny] = next.g;
                    fringe.push(next);
                }
            }
        }
    }
    return false;
}
bool DLS(int** M, pair<int,int> **parent, int w[], int m ,int n,
int sx, int sy, int tx, int ty, int limit){
    stack<state> fringe;
    state cur = {sx,sy,0,0};
    fringe.push(cur);
    parent[sx][sy] = {sx,sy};
    while (!fringe.empty()){
        cur = fringe.top(); 
        fringe.pop();
        if (cur.x == tx && cur.y == ty) { 
            cout << cur.g << endl;
            return true; 
        }
        if (cur.d>=limit) continue;
        for (int i = 0 ; i < 4 ; i++){
            int nx = cur.x + dx[i];
            int ny = cur.y + dy[i];
            if (valid2(M,m,n,nx,ny)) {
                state next = {nx,ny,cur.g+w[M[nx][ny]],cur.d+1};
                parent[nx][ny] = {cur.x,cur.y};
                fringe.push(next);
            }
        }
    }
    return false; 
}
bool IDS(int** M, pair<int,int> **parent, int w[], int m ,int n,
int sx, int sy, int tx, int ty){
    for (int i = 1; i < m*n; i++) if (DLS(M,parent,w,m,n,sx,sy,tx,ty,i)) return true;
    return false;
}
bool GBFS(int** M, bool **visited, pair<int,int> **parent, int w[], int m ,int n,
int sx, int sy, int tx, int ty, int type){
    priority_queue<state,vector<state>,cmp2> fringe; 
    state cur = {sx,sy,0,0,h(sx,sy,tx,ty,type)};
    fringe.push(cur); 
    while (!fringe.empty()){
        cur = fringe.top(); fringe.pop();
        if (visited[cur.x][cur.y]) continue;
        visited[cur.x][cur.y] = true;
        parent[cur.x][cur.y] = {cur.px,cur.py};
        if (cur.x == tx && cur.y == ty) { 
            cout << cur.g << endl;
            return true; 
        }
        for (int i = 0 ; i < 4 ; i++){
            int nx = cur.x + dx[i];
            int ny = cur.y + dy[i];
            if (valid1(M,visited,m,n,nx,ny)) {
                state next = {nx,ny,cur.g+w[M[nx][ny]],0,h(nx,ny,tx,ty,type),cur.x,cur.y};
                fringe.push(next);
            }
        }
    }
    return false; 
}
bool Astar(int** M, pair<int,int> **parent, int w[], int m ,int n,
int sx, int sy, int tx, int ty, int type){
    int dist[m][n]; 
    for (int i = 0 ; i < m ; i++){
        for (int j = 0 ; j < n ; j++){
            dist[i][j] = INT_MAX; 
        }
    }
    dist[sx][sy] = 0;
    priority_queue<state,vector<state>,cmp3> fringe; 
    state cur = {sx,sy,0,0,h(sx,sy,tx,ty,type)};
    fringe.push(cur);
    parent[sx][sy] = {sx,sy};
    while (!fringe.empty()){
        cur = fringe.top(); 
        fringe.pop();
        if (cur.g > dist[cur.x][cur.y]) continue;
        if (cur.x == tx && cur.y == ty) { 
            cout << cur.g << endl;
            return true; 
        }
        for (int i = 0 ; i < 4 ; i++){
            int nx = cur.x + dx[i];
            int ny = cur.y + dy[i];
            if (valid2(M,m,n,nx,ny)) {
                state next = {nx,ny,cur.g+w[M[nx][ny]],0,h(nx,ny,tx,ty,type)};
                if (dist[nx][ny]==INT_MAX||next.g<dist[nx][ny]) {
                    parent[nx][ny] = {cur.x,cur.y};
                    dist[nx][ny] = next.g;
                    fringe.push(next);
                }
            }
        }
    }
    return false;
}
int main(){
    int t; 
    cout << "Enter number of testcases: ";
    cin >> t; 
    while (t--){
        int m, n, tcnt, sx, sy, tx, ty; 
        wavg = 0;
        cout << "Enter maze size (m x n): " << endl;
        cout << "m = ";
        cin >> m;
        cout << "n = "; 
        cin >> n;
        cout << "Enter number of terrain types: ";
        cin >> tcnt; 
        int w[tcnt+1], **M = NULL;
        bool **visited = NULL;
        pair<int,int> **parent= NULL; 
        arrInit((void***)&M,m,n,0);
        arrInit((void***)&visited,m,n,1);
        arrInit((void***)&parent,m,n,2);
        cout <<  "Enter terrain costs" << endl; 
        for (int i = 1; i <= tcnt; i++) {
            cin >> w[i]; 
            wavg += w[i];
        }
        wavg/=tcnt;
        cout << "Enter maze" << endl;
        for (int i = 0 ; i < m ; i++){
            for (int j = 0 ; j < n ; j++){
                cin >> M[i][j];
                visited[i][j] = false;
                if (M[i][j]>tcnt){
                    cout << "invalid terrain";
                    return 0; 
                }
            }
        }
        cout << "Enter starting coordinates: "; 
        cin >> sx >> sy;
        cout  << "Enter target coordinates: "; 
        cin >> tx >> ty;
        cout  << "Enter search algorithm: ";
        string command; 
        cin >> command;
        bool solved = false;
        if (command=="BFS") solved = BFS(M,visited,parent,w,m,n,sx,sy,tx,ty);
        else if (command == "DFS") solved = DFS(M,visited,parent,w,m,n,sx,sy,tx,ty);
        else if (command == "UCS") solved = UCS(M,parent,w,m,n,sx,sy,tx,ty);
        else if (command == "DLS"){
            int limit;
            cin >> limit;
            solved = DLS(M,parent,w,m,n,sx,sy,tx,ty,limit);
        }
        else if (command == "IDS") solved = IDS(M,parent,w,m,n,sx,sy,tx,ty);
        else if (command == "Astar") {
            int type;
            cin >> type;
            solved = Astar(M,parent,w,m,n,sx,sy,tx,ty,type);
        }
        else if (command == "GBFS") {
            int type;
            cin >> type;
            solved = GBFS(M,visited,parent,w,m,n,sx,sy,tx,ty,type);
        }
        if (solved) trace(parent,tx,ty);
        else cout << "Solution not found"; 
        cout << endl;
        arrFree((void***)&M,m,0);
        arrFree((void***)&visited,m,1);
        arrFree((void***)&parent,m,2);
    }
    return 0;
}