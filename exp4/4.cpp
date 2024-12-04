#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <algorithm>
#include <climits>

// 定义图的结构
class Graph {
public:
    int numVertices;
    bool directed;
    // 邻接列表，每个顶点的邻居及边的权重
    std::vector<std::vector<std::pair<int, int>>> adjList;

    Graph(int vertices, bool isDirected = false) {
        numVertices = vertices;
        directed = isDirected;
        adjList.resize(vertices);
    }

    // 添加边
    void addEdge(int src, int dest, int weight = 1) {
        adjList[src].emplace_back(dest, weight);
        if (!directed) {
            adjList[dest].emplace_back(src, weight);
        }
    }

    // 打印图
    void printGraph() const {
        for (int i = 0; i < numVertices; ++i) {
            std::cout << "Vertex " << i << ":";
            for (auto &edge : adjList[i]) {
                std::cout << " -> (" << edge.first << ", " << edge.second << ")";
            }
            std::cout << std::endl;
        }
    }
};

// 广度优先搜索（BFS）
void BFS(const Graph &graph, int start) {
    std::vector<bool> visited(graph.numVertices, false);
    std::queue<int> q;

    visited[start] = true;
    q.push(start);

    std::cout << "BFS Traversal starting from vertex " << start << ": ";

    while (!q.empty()) {
        int current = q.front();
        q.pop();
        std::cout << current << " ";

        for (auto &neighbor : graph.adjList[current]) {
            if (!visited[neighbor.first]) {
                visited[neighbor.first] = true;
                q.push(neighbor.first);
            }
        }
    }
    std::cout << std::endl;
}

// 深度优先搜索（DFS）辅助函数
void DFSUtil(const Graph &graph, int vertex, std::vector<bool> &visited) {
    visited[vertex] = true;
    std::cout << vertex << " ";

    for (auto &neighbor : graph.adjList[vertex]) {
        if (!visited[neighbor.first]) {
            DFSUtil(graph, neighbor.first, visited);
        }
    }
}

// 深度优先搜索（DFS）
void DFS(const Graph &graph, int start) {
    std::vector<bool> visited(graph.numVertices, false);
    std::cout << "DFS Traversal starting from vertex " << start << ": ";
    DFSUtil(graph, start, visited);
    std::cout << std::endl;
}

// 最短路径算法（Dijkstra）
void Dijkstra(const Graph &graph, int start) {
    std::vector<int> distances(graph.numVertices, INT32_MAX);
    distances[start] = 0;

    // 使用优先级队列，存储 (距离, 顶点)
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<>> pq;
    pq.emplace(0, start);

    while (!pq.empty()) {
        int dist = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        // 避免重复处理
        if (dist > distances[u]) continue;

        for (auto &neighbor : graph.adjList[u]) {
            int v = neighbor.first;
            int weight = neighbor.second;

            if (distances[u] + weight < distances[v]) {
                distances[v] = distances[u] + weight;
                pq.emplace(distances[v], v);
            }
        }
    }

    std::cout << "Shortest distances from vertex " << start << ":\n";
    for (int i = 0; i < graph.numVertices; ++i) {
        if (distances[i] == INT32_MAX)
            std::cout << "Vertex " << i << ": INF\n";
        else
            std::cout << "Vertex " << i << ": " << distances[i] << "\n";
    }
}

// 边结构体
struct Edge {
    int src, dest, weight;
};

// 并查集（Union-Find）结构
class UnionFind {
public:
    std::vector<int> parent;
    UnionFind(int size) : parent(size) {
        for(int i=0;i<size;++i) parent[i] = i;
    }

    int findSet(int x) {
        if(parent[x] != x)
            parent[x] = findSet(parent[x]); // 路径压缩
        return parent[x];
    }

    void unionSet(int x, int y) {
        int setX = findSet(x);
        int setY = findSet(y);
        if(setX != setY)
            parent[setY] = setX;
    }
};

// Kruskal算法
void KruskalMST(const Graph &graph) {
    std::vector<Edge> edges;
    for(int u=0; u<graph.numVertices; ++u){
        for(auto &pair : graph.adjList[u]){
            int v = pair.first;
            int w = pair.second;
            if(u < v){ // 避免重复边
                edges.push_back(Edge{u, v, w});
            }
        }
    }

    // 按权重排序
    std::sort(edges.begin(), edges.end(), [](const Edge &a, const Edge &b) -> bool {
        return a.weight < b.weight;
    });

    UnionFind uf(graph.numVertices);
    std::vector<Edge> mst;
    int totalWeight = 0;

    for(auto &edge : edges){
        if(uf.findSet(edge.src) != uf.findSet(edge.dest)){
            uf.unionSet(edge.src, edge.dest);
            mst.push_back(edge);
            totalWeight += edge.weight;
        }
    }

    std::cout << "Minimum Spanning Tree (Kruskal's Algorithm):\n";
    for(auto &edge : mst){
        std::cout << edge.src << " -- " << edge.dest << " == " << edge.weight << "\n";
    }
    std::cout << "Total Weight: " << totalWeight << "\n";
}

// 优先级队列测试
void testPriorityQueue() {
    // 创建一个最小堆优先级队列
    std::priority_queue<int, std::vector<int>, std::greater<>> minHeap;
    minHeap.push(10);
    minHeap.push(5);
    minHeap.push(20);
    minHeap.push(1);

    std::cout << "Priority Queue (Min-Heap) elements: ";
    while(!minHeap.empty()){
        std::cout << minHeap.top() << " ";
        minHeap.pop();
    }
    std::cout << std::endl;
}

// 测试 BFS 和 DFS
void testBFSandDFS() {
    // 创建一个无向图
    Graph g(6, false);
    g.addEdge(0, 1);
    g.addEdge(0, 2);
    g.addEdge(1, 3);
    g.addEdge(1, 4);
    g.addEdge(2, 4);
    g.addEdge(3, 5);
    g.addEdge(4, 5);

    g.printGraph();

    BFS(g, 0);
    DFS(g, 0);
}

// 测试最短路径和最小生成树
void testShortestPathAndMST() {
    // 创建一个有向加权图
    Graph g(5, false);
    g.addEdge(0, 1, 10);
    g.addEdge(0, 2, 6);
    g.addEdge(0, 3, 5);
    g.addEdge(1, 3, 15);
    g.addEdge(2, 3, 4);

    g.printGraph();

    Dijkstra(g, 0);
    KruskalMST(g);
}

int main() {
    std::cout << "=== 测试 BFS 和 DFS ===\n";
    testBFSandDFS();

    std::cout << "\n=== 测试最短路径和最小生成树 ===\n";
    testShortestPathAndMST();

    std::cout << "\n=== 测试优先级队列 ===\n";
    testPriorityQueue();

    return 0;
}
