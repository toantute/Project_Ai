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
using std::min;

int dx[] = {-1,0,1,0}; 
int dy[] = {0,1,0,-1};
typedef struct state{
    int x, y, px, py, d, g ;
    double h, f;
}state;
typedef struct cmp{
    bool operator()(state a , state b){
        return a.f > b.f;
    }
}cmp;

double h(int x, int y, int tx, int ty, int wmin, int type, double mul){
    int disx = x > tx ? x-tx : tx - x;
    int disy = y > ty ? y-ty : ty - y;
    if (type) return sqrt(disx*disx+disy*disy) * wmin * mul;
    else return (disx+disy) * wmin * mul;
}

typedef struct sim{ 
    int m, n, sx, sy, tx, ty;//dimensions and coords
    int **M, **dis, **dmin, type, l, lcap, wmin = INT_MAX; //maze, niche arrays and variables
    double mul;
    vector<int> w;
    pair<int,int> **p; //parent for tracing
    stack<state> st; 
    queue<state> q; 
    priority_queue<state,vector<state>,cmp> pq;
    bool **visited, done, found, reachedLimit;
    string a, id;

    sim(string _id){
        done = found = false;
        id = _id;
        int tcnt;
        cout << "Enter maze size (m x n): " << endl;
        cout << "m = ";
        cin >> m;
        cout << "n = "; 
        cin >> n;
        cout << "Enter number of terrain types: ";
        cin >> tcnt; 
        w = vector<int>(tcnt+1);
        M = new int*[m];
        int *tmp = new int[m*n]; 
        for (int i = 0 ; i < m; i++,tmp+=n) M[i] = tmp;
        cout <<  "Enter terrain costs" << endl; 
        for (int i = 1; i <= tcnt; i++) {
            cin >> w[i]; wmin = min(wmin,w[i]);
        }
        cout << "Enter maze" << endl;
        for (int i = 0 ; i < m ; i++){
            for (int j = 0 ; j < n ; j++){
                cin >> M[i][j];
                if (M[i][j]>tcnt){
                    cout << "invalid terrain";
                }
            }
        }
        cout << "Enter starting coordinates: "; 
        cin >> sx >> sy;
        cout  << "Enter target coordinates: "; 
        cin >> tx >> ty;
        cout  << "Enter search algorithm: ";
        cin >> a;

        if (a=="DLS"){
            cout << "Enter depth limit: ";
            cin >> l;
        }
        else if (a=="GBFS"||a=="Astar"){
            cout << "Choose heuristics (0 for manhattan distance, 1 for euclidean distance): "; 
            cin >> type; 
            cout << "Enter heuristics multiplier: ";
            cin >> mul; 
        }

        p = new pair<int,int>*[m];
        pair<int,int> *tmp1 = new pair<int,int>[m*n];
        for (int i = 0 ; i < m ; i++,tmp1+=n) p[i] = tmp1; 

        for (int i = 0 ; i < m ; i++){
            for (int j = 0 ; j < n ; j++){
                p[i][j] = {-1,-1};
            }
        }

        init();
    }

    void init(){
        //visited[][] prevents re-expanding the same node
        if (a=="BFS"||a=="DFS"||a=="GBFS"){
            visited = new bool*[m];
            bool *tmp2 = new bool[m*n];
            for (int i = 0 ; i < m ; i++,tmp2+=n) visited[i] = tmp2; 

        for (int i = 0 ; i < m ; i++){
            for (int j = 0 ; j < n ; j++){
            visited[i][j] = false;
            }
        }
        }

        if (a=="BFS"){
        q.push({sx,sy,sx,sy,0,0,0,0}); 
        visited[sx][sy] = true;
        p[sx][sy] = {sx,sy};
        }

        else if (a=="DFS"){
        st.push({sx,sy,sx,sy,0,0,0,0}); 
        visited[sx][sy] = true;
        p[sx][sy] = {sx,sy};
        }

        //dis[][] tracks best known g-cost; allows re-expansion if cheaper path found
        else if (a=="UCS"||a=="Astar"){
        dis = new int*[m]; 
        int *tmp = new int[m*n];
        for (int i = 0 ; i < m ; i++, tmp+=n) dis[i] = tmp;  

        for (int i = 0 ; i < m ; i++){
            for (int j = 0 ; j < n ; j++){
                dis[i][j] = INT_MAX;
            }
        }
        
        state first = {sx,sy,sx,sy,0,0,0,0};
        if (a=="Astar") {
            first.h = h(sx,sy,tx,ty,wmin,type,mul);
            first.f = first.h;
        }
        pq.push(first);
        dis[sx][sy] = 0;
        p[sx][sy] = {sx,sy};
        }

        //dmin[][] tracks shallowest depth seen; allows re-expansion if shallower path found
        else if (a=="DLS"||a=="IDS"){
        dmin = new int*[m];
        int *tmp = new int[m*n];
        for (int i = 0 ; i < m ; i++, tmp+=n) dmin[i] = tmp;  

        for (int i = 0 ; i < m ; i++){
            for (int j = 0 ; j < n ; j++){
                dmin[i][j] = INT_MAX;
            }
        }

        st.push({sx,sy,sx,sy,0,0,0,0}); 
        p[sx][sy] = {sx,sy};
        dmin[sx][sy] = 0;

        reachedLimit = false;

        if (a=="IDS") {
            l = 0; lcap = m*n-1; 
        }
        }

        else if (a=="GBFS"){
        state first = {sx,sy,sx,sy,0,0,h(sx,sy,tx,ty,wmin,type,mul)};
        first.f = first.h;
        pq.push(first); 
        visited[sx][sy] = true;
        p[sx][sy] = {sx,sy};
        }
    }

    ~sim(){
    delete []p[0]; 
    delete []p; 
    
    delete []M[0];
    delete []M;
    if (a=="UCS"||a=="Astar") {
        delete []dis[0];
        delete []dis;
    }
    else if (a=="DLS"||a=="IDS"){
        delete []dmin[0];
        delete []dmin;
    }
    else if (a=="BFS"||a=="DFS"||a=="GBFS"){
        delete []visited[0];
        delete []visited;
    }
    }


    void trace(int x, int y){
    if (p[x][y].first != x || p[x][y].second != y)
    trace(p[x][y].first,p[x][y].second);
    cout << "("<< x << "," << y << ")" << " ";
    }
    
    void reset(){ //IDS only
    l++;

    
    while (!st.empty()) st.pop();

    for (int i = 0 ; i < m ; i++){
        for (int j = 0 ; j < n ; j++){
            dmin[i][j] = INT_MAX;
            p[i][j] = {-1,-1};
        }
    }

    st.push({sx,sy,sx,sy,0,0,0,0});
    dmin[sx][sy] = 0;
    p[sx][sy] = {sx,sy}; 

    done = reachedLimit = false;
}

}sim;

bool valid(sim &s, int x, int y){ // BFS/DFS/GBFS: in-bounds, non-wall, unvisited
    return x>=0&&y>=0&&x<s.m&&y<s.n&&s.M[x][y]!=0&&!s.visited[x][y];
}
bool valid(sim &s, int x, int y, int g){ // UCS/A*: in-bounds, non-wall, cheaper than known cost
    return x>=0&&y>=0&&x<s.m&&y<s.n&&s.M[x][y]!=0&&g+s.w[s.M[x][y]]<s.dis[x][y];
}
bool valid(int d, sim&s,int x, int y){ // DLS/IDS: in-bounds, non-wall, within depth limit and shallower than dmin
    return x>=0&&y>=0&&x<s.m&&y<s.n&&s.M[x][y]!=0&&d<=s.l&&d<s.dmin[x][y];
}
void BFS(sim &s){

    if (s.q.empty()) {
        s.done = true; 
        cout << "No solution found" << endl; 
        return;
    }
    
    state cur = s.q.front(); s.q.pop(); 
    int x = cur.x, y = cur.y;
    cout << "[" << s.id << "] " << "Expanding Node (" << x << "," << y << ")" << endl; 
    if (x==s.tx&&y==s.ty){
        s.found = s.done = true; 
        s.trace(s.tx,s.ty);
        cout << endl;
        return;
    }

    for (int i = 0 ; i < 4 ; i++){
        int nx = x + dx[i]; 
        int ny = y + dy[i]; 
        if (valid(s,nx,ny)){
            state next = {nx,ny,x,y,cur.d+1,cur.g+s.w[s.M[nx][ny]]};
            s.visited[nx][ny] = true; s.p[nx][ny] = {x,y};
            s.q.push(next); 
        }
    }


}

void DFS(sim &s){

    if (s.st.empty()) {
        s.done = true; 
        cout << "No solution found" << endl; 
        return;
    }
    
    state cur = s.st.top(); s.st.pop(); 
    int x = cur.x, y = cur.y;
    cout << "[" << s.id << "] " << "Expanding Node (" << x << "," << y << ")" << endl; 
    if (x==s.tx&&y==s.ty){
        s.found = s.done = true; 
        s.trace(s.tx,s.ty);
        cout << endl;
        return;
    }

    for (int i = 0 ; i < 4 ; i++){
        int nx = x + dx[i]; 
        int ny = y + dy[i]; 
        if (valid(s,nx,ny)){
            state next = {nx,ny,x,y,cur.d+1,cur.g+s.w[s.M[nx][ny]]};
            s.visited[nx][ny] = true; s.p[nx][ny] = {x,y};
            s.st.push(next); 
        }
    }
}

void UCS(sim&s){ //dijkstra-like logic, skip stale nodes by recursing until expanded 1 node
    
    if (s.pq.empty()){
        s.done = true;
        cout <<"No solution found" << endl; 
        return;
    }

    state cur = s.pq.top(); s.pq.pop();
    int x = cur.x, y = cur.y;
    if (cur.g > s.dis[x][y]) {
        UCS(s);
        return;
    } 

    cout << "[" << s.id << "] " << "Expanding Node (" << x << "," << y << ")" << endl; 
    s.p[x][y] = {cur.px,cur.py};
    if (x==s.tx&&y==s.ty){
        s.found = s.done = true; 
        s.trace(s.tx,s.ty);
        cout << endl;
        return;
    }

    for (int i = 0 ; i < 4 ; i++){
        int nx = x + dx[i]; 
        int ny = y + dy[i]; 
        
        if (valid(s,nx,ny,cur.g)){
            state next = {nx,ny,x,y,cur.d+1,cur.g+s.w[s.M[nx][ny]]};
            next.f = s.dis[nx][ny] = next.g;
            s.p[nx][ny] = {x,y};
            s.pq.push(next); 
        }
    }
}

void DLS(sim &s){ //use dmin to track shallowest depth, only reexplore if found a shallower path

    if (s.st.empty()) {
        s.done = true; 
        cout << "No solution found" << endl; 
        return;
    }
    
    state cur = s.st.top(); s.st.pop(); if (cur.d == s.l) s.reachedLimit = true;
    int x = cur.x, y = cur.y;
    cout << "[" << s.id << "] " << "Expanding Node (" << x << "," << y << ")" << endl; 
    if (x==s.tx&&y==s.ty){
        s.found = s.done = true; 
        s.trace(s.tx,s.ty);
        cout << endl;
        return;
    }

    for (int i = 0 ; i < 4 ; i++){
        int nx = x + dx[i]; 
        int ny = y + dy[i]; 
        if (valid(cur.d+1,s,nx,ny)){
            state next = {nx,ny,x,y,cur.d+1,cur.g+s.w[s.M[nx][ny]]};
            s.dmin[nx][ny] = next.d;
            s.p[nx][ny] = {x,y};
            s.st.push(next); 
        }
    }
}

void IDS(sim &s){
    DLS(s);

    if (s.done&&!s.found){
        if (!s.reachedLimit||s.l==s.lcap){ //exit early if no new node is found
            cout << "No solution found" << endl; 
            return;
        }
        else {
            cout << "Increasing depth limit to " << s.l+1 << endl;
            s.reset();
        }
    }
}

void GBFS(sim &s){
    if (s.pq.empty()) {
        s.done = true; 
        cout << "No solution found" << endl; 
        return;
    }
    
    state cur = s.pq.top(); s.pq.pop(); 
    int x = cur.x, y = cur.y;
    cout << "[" << s.id << "] " << "Expanding Node (" << x << "," << y << ")" << endl; 
    if (x==s.tx&&y==s.ty){
        s.found = s.done = true; 
        s.trace(s.tx,s.ty);
        cout << endl;
        return;
    }

    for (int i = 0 ; i < 4 ; i++){
        int nx = x + dx[i]; 
        int ny = y + dy[i]; 
        if (valid(s,nx,ny)){
            state next = {nx,ny,x,y,cur.d+1,cur.g+s.w[s.M[nx][ny]],h(nx,ny,s.tx,s.ty,s.wmin,s.type,s.mul)};
            next.f = next.h;
            s.visited[nx][ny] = true; s.p[nx][ny] = {x,y};
            s.pq.push(next); 
        }
    }
}

void Astar(sim&s){ //UCS but f = g + h 
    
    if (s.pq.empty()){
        s.done = true;
        cout <<"No solution found" << endl; 
        return;
    }

    state cur = s.pq.top(); s.pq.pop();
    int x = cur.x, y = cur.y;
    if (cur.g > s.dis[x][y]) {
        Astar(s);
        return;
    } 

    cout << "[" << s.id << "] " << "Expanding Node (" << x << "," << y << ")" << endl; 
    s.p[x][y] = {cur.px,cur.py};
    if (x==s.tx&&y==s.ty){
        s.found = s.done = true; 
        s.trace(s.tx,s.ty);
        cout << endl;
        return;
    }

    for (int i = 0 ; i < 4 ; i++){
        int nx = x + dx[i]; 
        int ny = y + dy[i]; 
        
        if (valid(s,nx,ny,cur.g)){
            state next = {nx,ny,x,y,cur.d+1,cur.g+s.w[s.M[nx][ny]],h(nx,ny,s.tx,s.ty,s.wmin,s.type,s.mul)};
            next.f = s.dis[nx][ny] = next.g;
            next.f+=next.h;
            s.p[nx][ny] = {x,y};
            s.pq.push(next); 
        }
    }
}

void run(sim &s){
    if (s.done) return;
    if (s.a=="BFS") BFS(s);
    else if (s.a=="DFS") DFS(s);
    else if (s.a=="UCS") UCS(s);
    else if (s.a=="DLS") DLS(s);
    else if (s.a=="IDS") IDS(s); 
    else if (s.a=="GBFS") GBFS(s);
    else if (s.a=="Astar") Astar(s);
}
int main(){
    int t; 
    cout << "Enter number of testcases: ";
    cin >> t; 
    while (t--){
        int mode;
        cout << "Choose mode (1 for single simulation, 2 for double simulation): "; 
        cin >> mode; 
        //control flow, p = run until done, n = expand one node at a time
        if (mode == 1){
            sim s = sim("S");
            while (!s.done){
                char command;
                cin >> command; 
                if (command == 'n') run(s); 
                if (command == 'p' ) {while (!s.done) run(s);}
            }
        }
        else if (mode==2){
            cout << "Input simulation 1:" << endl; 
            sim s1 = sim("S1");
            cout << "Input simulation 2:" << endl;
            sim s2 = sim("S2");
            while (!s1.done || !s2.done){
                char command;
                cin >> command; 
                if (command == 'n') {
                    run(s1);
                    run(s2);
                }
                else if (command == 'p') {
                while  (!s1.done||!s2.done){
                    run(s1);
                    run(s2);
                }
            }
            }
        }
    }
    return 0;
}