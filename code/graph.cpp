/**************************************************************************
 * Authors: Makenzie Kadjeski, Jeny Thomas, Carrie Houston
 * Purpose: Applying 4 different algorithms- BFS, DFS, A*, Bidirectional- 
 *          to find the shortest path between two destinations on 
 *          campus. We also compare the time it takes for each algorithm
 *          to compute the results.
 * Compile: g++ -O2 graph.cpp -o run
 **************************************************************************/



#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <unordered_map>
#include <tuple>
#include <algorithm>
#include <cmath>
#include <chrono>

using namespace std;

// ======= GLOBAL GRAPH =======
vector<vector<int>> graph = {
    //LARGE
    // {0,	0,	1,	1,	1,	1,	0,	0,	0,	0,	0,	0,	0,	0,	1,	1},
    // {0,	0,	0,	1,	1,	1,	1,	0,	1,	1,	0,	0,	0,	0,	0,	0},
    // {1,	1,	0,	0,	1,	1,	1,	0,	0,	1,	1,	0,	0,	1,	0,	0},
    // {0,	1,	0,	1,	0,	0,	0,	0,	0,	1,	1,	1,	0,	1,	0,	0},
    // {1,	1,	0,	1,	0,	1,	0,	0,	0,	0,	1,	0,	0,	0,	0,	0},
    // {1,	1,	1,	1,	1,	0,	0,	0,	1,	1,	0,	0,	0,	0,	0,	0},
    // {1,	0,	0,	0,	0,	1,	0,	1,	1,	1,	0,	1,	0,	1,	1,	1},
    // {0,	0,	1,	0,	0,	0,	0,	0,	1,	1,	0,	1,	1,	1,	1,	1},
    // {0,	1,	1,	0,	0,	1,	0,	1,	0,	1,	0,	1,	0,	1,	0,	0},
    // {0,	1,	1,	1,	0,	1,	1,	1,	1,	0,	1,	1,	0,	1,	0,	0},
    // {0,	0,	0,	1,	1,	0,	1,	0,	0,	1,	0,	1,	0,	0,	0,	0},
    // {0,	0,	1,	0,	0,	0,	1,	1,	1,	1,	1,	0,	1,	1,	0,	0},
    // {0,  0,	0,	0,	0,	0,	0,	1,	0,	0,	0,	1,	0,	1,	0,	1},
    // {0,	0,	1,	1,	0,	0,	1,	1,	1,	1,	0,	1,	1,	0,	0,	0},
    // {1,	0,	1,	0,	0,	0,	0,	1,	0,	0,	0,	0,	0,	0,	0,	1},
    // {1,	0,	1,	0,	0,	0,	0,	1,	0,	0,	0,	0,	1,	0,	1,	0}






    //MEDIUM
    // {0, 1, 1, 1, 1, 1, 0, 0},
    // {1, 0, 0, 1, 0, 1, 1, 0},
    // {1, 0, 0, 0, 0, 1, 0, 1},
    // {1, 1, 0, 0, 1, 1, 1, 0},
    // {1, 0, 0, 1, 0, 0, 0, 0},
    // {1, 1, 1, 1, 0, 0, 0, 0},
    // {0, 1, 0, 1, 0, 1, 0, 0},
    // {0, 0, 1, 0, 0, 0, 0, 0}

    //SMALL
    {0, 1, 1, 0},
    {1, 0, 1, 1},
    {1, 1, 0, 1},
    {0, 1, 1, 0}

};



// ======= BFS =======
vector<int> BFS(int start, int goal) {
    queue<int> q;
    vector<int> parent(graph.size(), -1);
    vector<bool> visited(graph.size(), false);

    q.push(start);
    visited[start] = true;

    while (!q.empty()) {
        int node = q.front(); q.pop();
        if (node == goal) break;

        for (int neighbor = 0; neighbor < (int)graph[node].size(); neighbor++) {
            if (graph[node][neighbor] != 0 && !visited[neighbor]) {
                visited[neighbor] = true;
                parent[neighbor] = node;
                q.push(neighbor);
            }
        }
    }

    vector<int> path;
    if (!visited[goal]) return path;

    for (int cur = goal; cur != -1; cur = parent[cur])
        path.push_back(cur);
    reverse(path.begin(), path.end());
    return path;
}

// ======= DFS =======
vector<int> DFS(int start, int goal) {
    stack<int> st;
    vector<int> parent(graph.size(), -1);
    vector<bool> visited(graph.size(), false);

    st.push(start);
    visited[start] = true;

    while (!st.empty()) {
        int node = st.top(); st.pop();
        if (node == goal) break;

        for (int neighbor = (int)graph[node].size() - 1; neighbor >= 0; neighbor--) {
            if (graph[node][neighbor] != 0 && !visited[neighbor]) {
                visited[neighbor] = true;
                parent[neighbor] = node;
                st.push(neighbor);
            }
        }
    }

    vector<int> path;
    if (!visited[goal]) return path;
    for (int cur = goal; cur != -1; cur = parent[cur])
        path.push_back(cur);
    reverse(path.begin(), path.end());
    return path;
}

// ======= A* =======
double heuristic(int a, int b) {
    // Simple heuristic: assume node number distance
    return abs(a - b);
}

vector<int> AStar(int start, int goal) {
    using State = tuple<double, double, int>;
    struct Compare {
        bool operator()(const State& a, const State& b) const {
            return get<0>(a) > get<0>(b); // min-heap by f
        }
    };

    priority_queue<State, vector<State>, Compare> pq;
    unordered_map<int, int> parent;
    unordered_map<int, double> gScore;

    gScore[start] = 0;
    pq.push({heuristic(start, goal), 0.0, start});
    parent[start] = -1;

    while (!pq.empty()) {
        auto [f, g, node] = pq.top(); pq.pop();
        if (node == goal) break;

        for (int neighbor = 0; neighbor < (int)graph[node].size(); neighbor++) {
            if (graph[node][neighbor] != 0) {
                double newG = g + 1.0;
                if (!gScore.count(neighbor) || newG < gScore[neighbor]) {
                    gScore[neighbor] = newG;
                    double nf = newG + heuristic(neighbor, goal);
                    pq.push({nf, newG, neighbor});
                    parent[neighbor] = node;
                }
            }
        }
    }

    vector<int> path;
    if (!parent.count(goal)) return path;
    for (int cur = goal; cur != -1; cur = parent[cur])
        path.push_back(cur);
    reverse(path.begin(), path.end());
    return path;
}

// ======= Bidirectional Search =======
vector<int> Bidirectional(int start, int goal) {
    unordered_map<int, int> fParent, bParent;
    queue<int> fQ, bQ;
    fQ.push(start);
    bQ.push(goal);
    fParent[start] = -1;
    bParent[goal] = -1;

    while (!fQ.empty() && !bQ.empty()) {
        int f = fQ.front(); fQ.pop();
        if (bParent.count(f)) {
            // meet in the middle
            vector<int> path1, path2;
            for (int cur = f; cur != -1; cur = fParent[cur]) path1.push_back(cur);
            reverse(path1.begin(), path1.end());
            for (int cur = bParent[f]; cur != -1; cur = bParent[cur]) path2.push_back(cur);
            path1.insert(path1.end(), path2.begin(), path2.end());
            return path1;
        }

        for (int neighbor = 0; neighbor < (int)graph[f].size(); neighbor++) {
            if (graph[f][neighbor] != 0 && !fParent.count(neighbor)) {
                fParent[neighbor] = f;
                fQ.push(neighbor);
            }
        }

        int b = bQ.front(); bQ.pop();
        if (fParent.count(b)) {
            // meet in the middle
            vector<int> path1, path2;
            for (int cur = fParent[b]; cur != -1; cur = fParent[cur]) path1.push_back(cur);
            reverse(path1.begin(), path1.end());
            for (int cur = b; cur != -1; cur = bParent[cur]) path2.push_back(cur);
            path1.insert(path1.end(), path2.begin(), path2.end());
            return path1;
        }

        for (int neighbor = 0; neighbor < (int)graph[b].size(); neighbor++) {
            if (graph[b][neighbor] != 0 && !bParent.count(neighbor)) {
                bParent[neighbor] = b;
                bQ.push(neighbor);
            }
        }
    }
    return {};
}

// ======= Print Function =======
void printPath(const vector<int>& path, const string& name) {
    cout << name << " Path (" << max(0, (int)path.size() - 1) << " steps): ";
    for (int p : path) cout << p+1 << " ";
    cout << endl;
}

// ======= MAIN =======
int main() {
    int start = 0;
    int goal = 15;
    char choice;



    do
    {
            cout << "\nHere is the list of locations in order.";
            cout << "\n\t1. Ashraf Islam Engineering Building";
            cout << "\n\t2. Bruner Hall";
            cout << "\n\t3. Library";
            cout << "\n\t4. RUC";
            cout << "\n\t5. Parking Lot";
            cout << "\n\t6. Prescott Hall";
            cout << "\n\t7. LSC";
            cout << "\n\t8. Stonecipher";
            cout << "\n\t9. Brown Hall";
            cout << "\n\t10. Clement Hall";
            cout << "\n\t11. BFA";
            cout << "\n\t12. Mem Gym";
            cout << "\n\t13. Bell Hall";                
            cout << "\n\t14. Derryberry";
            cout << "\n\t15. Fit";
            cout << "\n\t16. Ray Morris";        

            cout << "\n\nWhere would you like to start? Enter 1-16: ";
            cin >> start;
            start--;

            cout << "\nWhere would you like to end? Enter 1-16: ";
            cin >> goal;
            goal--;

            // --- BFS ---
            auto bfs_start = chrono::high_resolution_clock::now();
            auto bfs_path = BFS(start, goal);
            auto bfs_end = chrono::high_resolution_clock::now();
            auto bfs_time = chrono::duration_cast<chrono::microseconds>(bfs_end - bfs_start).count();
            printPath(bfs_path, "BFS");
            cout << "BFS Time: " << bfs_time << " microseconds\n\n";

            // --- DFS ---
            auto dfs_start = chrono::high_resolution_clock::now();
            auto dfs_path = DFS(start, goal);
            auto dfs_end = chrono::high_resolution_clock::now();
            auto dfs_time = chrono::duration_cast<chrono::microseconds>(dfs_end - dfs_start).count();
            printPath(dfs_path, "DFS");
            cout << "DFS Time: " << dfs_time << " microseconds\n\n";

            // --- A* ---
            auto a_start = chrono::high_resolution_clock::now();
            auto a_path = AStar(start, goal);
            auto a_end = chrono::high_resolution_clock::now();
            auto a_time = chrono::duration_cast<chrono::microseconds>(a_end - a_start).count();
            printPath(a_path, "A*");
            cout << "A* Time: " << a_time << " microseconds\n\n";

            // --- Bidirectional ---
            auto bi_start = chrono::high_resolution_clock::now();
            auto bi_path = Bidirectional(start, goal);
            auto bi_end = chrono::high_resolution_clock::now();
            auto bi_time = chrono::duration_cast<chrono::microseconds>(bi_end - bi_start).count();
            printPath(bi_path, "Bidirectional");
            cout << "Bidirectional Search Time: " << bi_time << " microseconds\n\n";


            cout << "\n\nWould you like to go again? Enter 'y' or 'n': ";
            cin >> choice;
    }while(tolower(choice) != 'n');
    
   
    return 0;
}

