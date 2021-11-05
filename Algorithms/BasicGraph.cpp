#include <bits/stdc++.h>
using namespace std;
#define rep(i, n) for (int i = 0; i < (n); i++)
#define ll long long
const int N = 2e5+22;
const int F = 1e9+22;
const ll INF = 2e15+22;

class Edge
{
public:
	int u, v;
    ll d;
	Edge(int uu, int vv, int dd = 0) { u = uu; v = vv; d = dd; }
};

// Weighted Graph
class Graph1
{
public:
	int n, m;
	vector<Edge> edges;
    vector<vector<pair<int, ll>>> adj;

	void get_weighted_graph()
	{
		cin >> n >> m;
        rep(i, m)
        {
            int u, v, d;
            cin >> u >> v >> d;
            u--, v--;
            edges.push_back(Edge(u, v, d));
        }
	}

    void get_weighted_graph2()
    {
        cin >> n >> m;
        adj = vector<vector<pair<int, ll>>>(n);
        rep (i, m)
        {
            int u, v, d;
            cin >> u >> v >> d;
            u--, v--;
            adj[u].push_back({v, d});
            adj[v].push_back({u, d});
        }
    }

    void get_graph()
	{
		cin >> n >> m;
        rep(i, m)
        {
            int u, v, d;
            cin >> u >> v;
            u--, v--;
            edges.push_back(Edge(u, v));
        }
	}

    /* 
        computes shortest paths from a single source vertex to all of the other vertices 
        in a weighted digraph
        returns true if graph has negative cycle
    */
    bool bellman_ford(int starting_node)
    {
        get_weighted_graph();
        vector<ll> dist(n, INF);
        dist[starting_node] = 0;
        
        rep(i, n)
        {
            for (auto& e : edges)
            {
                if (dist[e.v] > dist[e.u]+e.d)
                {
                    if (i == n-1)
                        return true; // negative cycle
                    dist[e.v] = dist[e.u]+e.d;   
                }
            }
        }
        return false;
    }

    //  all-pairs shortest path algorithm
    vector<vector<ll>> floyd_warshall(bool get_input_from_user=false)
    {
        if (get_input_from_user)
            get_weighted_graph();
        vector<vector<ll>> dist(n, vector<ll>(n, INF));
        rep(i,n) 
            dist[i][i]=0;
        for (auto& e : edges)
            dist[e.u][e.v] = e.d;

        rep(k,n) rep(i,n) rep(j,n) 
            dist[i][j]=min(dist[i][j], dist[i][k]+dist[k][j]);
    }

    vector<ll> dijkstra(int v, bool get_input_from_user=false)
    {
        if (get_input_from_user)
            get_weighted_graph2();
        vector<bool> visited(n, false);
        vector<ll> dist(n, INF);
        
        dist[v] = 0;
        priority_queue<pair<int,int>, vector<pair<int,int>>, less<pair<int,int>>> pq;
        pq.push({dist[v], v});

        while (!pq.empty())
        {
            int u = pq.top().second;
            pq.pop();

            if(visited[u])
                continue;
            visited[u] = true;
            for(auto p : adj[u]) //adj[v][i] = pair(vertex, weight)
                if(dist[p.first] > dist[u] + p.second)
                {
                    dist[p.first] = dist[u] + p.second;
                    pq.push({dist[p.first], p.first});
                }
        }

        return dist;
    }

    void dijkstra_set(int v)
    {
        get_weighted_graph2();
        vector<bool> visited(n, false);
        vector<ll> dist(n, INF);
        
        dist[v] = 0;
        set<pair<int,int>> s;
        s.insert({dist[v], v});
        while (!s.empty())
        {
            int u = (*s.begin()).second;
            s.erase(s.begin());
            for(auto p : adj[u]) //adj[v][i] = pair(vertex, weight)
                if(dist[p.first] > dist[u] + p.second){
                    s.erase({dist[p.first], p.first});
                    dist[p.first] = dist[u] + p.second;
                    s.insert({dist[p.first], p.first});
                }
        }
    }

    // computes single-source shortest paths in a weighted directed graph.
    // particularly suitable for graphs that contain negative-weight edges
    void shortestPathFaster(int starting_vertice)
    {
        get_weighted_graph2();
        vector<bool> in_queue(n, false);
        vector<ll> dist(n, INF);

        dist[starting_vertice] = 0;

        queue<int> q;
        q.push(starting_vertice);
        in_queue[starting_vertice] = true;

        while (!q.empty()) {
            int u = q.front();
            q.pop();
            in_queue[u] = false;

            for (auto& p : adj[u])
            {
                int v = p.first;
                int weight = p.second;

                if (dist[v] > dist[u] + weight) {
                    dist[v] = dist[u] + weight;
                    if (!in_queue[v]) 
                    {
                        q.push(v);
                        in_queue[v] = true;
                    }
                }
            }
        }
    }

    vector<ll> bfs(int v, bool get_input_from_user = false)
    {
        if (get_input_from_user)
        {
            get_graph();
            for (auto& e : edges)
                e.d = 1;
        }
        return dijkstra(v);
    }

    int find_farthest_vertice(vector<ll> dist)
    {
        int farthest_v = 0;
        for (int i=0; i<dist.size(); i++)
            if (dist[farthest_v] < dist[i])
                farthest_v = i;
        return farthest_v;
    }

    int find_diameter()
    {
        vector<ll> dist = bfs(0, true);
        int far_v = find_farthest_vertice(dist);
        dist = bfs(far_v, false);
        far_v = find_farthest_vertice(dist);
        return dist[far_v];
    }


    // Source: https://codeforces.com/blog/entry/17974
    // returns { radius, diameter, center_of_graph}
    tuple<int, int, vector<int>> find_center_fw()
    {
        get_graph();
        for (auto& e : edges)
            e.d = 1;
        vector<vector<ll>> dist = floyd_warshall(false);
        
        ll rad = F;
        ll diam = 0;
        vector<ll> e(n, 0);
        vector<int> centers;
        // Counting values of eccentricity
        rep(i, n) rep(j, n)
            e[i] = max(e[i], dist[i][j]);
    
        rep(i, n)
        {
            rad = min(rad, e[i]);
            diam = max(diam, e[i]);
        }

        rep(i, n)
            if (e[i] == rad) 
                centers.push_back(i);

        return {rad, diam, centers};
    }

    // Tree (set method)
    pair<int,int> find_center()
    {
        set<int> G[N];
        get_graph();
        for (auto& e : edges)
        {
            G[e.u].insert(e.v);
            G[e.v].insert(e.u);
        }
        set<pair<int,int>> s;
        rep(i,n) 
            s.insert({G[i].size(), i});
        
        while (s.size()>2)
        {
            int number_of_leaves=0;
            auto it = s.begin();
            vector<pair<int, int>> vec;
            
            while(it->first==1)
            {
                number_of_leaves++;
                int v = it->second; // leaf
                int u = (*G[v].begin());
                vec.push_back({u, v});
                it++;
            }
            while(number_of_leaves--) 
                s.erase(s.begin());
            
            rep(i, vec.size())
            {
                int u=vec[i].first, v=vec[i].second;
                
                s.erase({G[u].size(), u});
                G[u].erase(v);
                s.insert({G[u].size(), u});
            }
        }
        if(s.size()==1)
            return {(*s.begin()).second, -1};
        else
        {
            int u = (*s.begin()).second;
            s.erase(s.begin());
            return {u, (*s.begin()).second};
        }
    }

    // find_center(another method)
    vector<int> find_center_2()
    {
        vector<int> degree(n, 0);
        vector<vector<bool>> G(n, vector<bool>(n, false));
        get_graph();
        for (auto& e : edges)
        {
            degree[e.u]++;
            degree[e.v]++;
            G[e.u][e.v] = G[e.v][e.u] = true;
        }
        queue<int> q;

        // Start from leaves
        rep(i, n)
            if (degree[i] == 1)
                q.push(i);

        int maxlevel = 0;
        vector<int> level(n, 0);
        vector<int> centers;

        while (!q.empty()) 
        {
            int v = q.front();
            q.pop();

            // Remove leaf and try to add its parent
            rep(i, n)
            {
                if (G[v][i]) {
                    degree[i]--;
                    
                    if (degree[i] == 1) {
                        q.push(i);
                        level[i] = level[v] + 1;
                        maxlevel = max(maxlevel, level[i]);
                    }
                }
            }
        }

        rep(i, n)
            if (level[i] == maxlevel)
                centers.push_back(i);
        return centers;
    }
};

// Normal Graph
class Graph2
{
public:
    int n, m;
    vector<vector<int>> edges;
    
    void get_input()
    {
        cin >> n >> m;
        edges = vector<vector<int>>(n);
    
        rep(i, m)
        {
            int u, v;
            cin >> u >> v;
            u--, v--;
            edges[u].push_back(v);
            edges[v].push_back(u);
        }
    }

    vector<bool> visited;
    vector<int> starting_time, finishing_time;
    int time;
    void dfs_sf_time(int u)
    {
        starting_time[u] = time++;
        visited[u] = true;
        for (auto& v : edges[u])
        {
            if (!visited[v])
                dfs_sf_time(v);
        }
        finishing_time[u] = time;
    }

    vector<int> parent, distance, upp;
    vector<pair<int,int>> cut_edges;
    void dfs_cut_edge(int v, int d=0)
    {
        visited[v] = true;
        distance[v] = d;
        upp[v] = d;

        for (auto& u : edges[v])
        {
            if (!visited[u])
            {
                parent[u] = v;
                dfs_cut_edge(u, d+1);

                if(upp[u] > distance[v]) 
                    cut_edges.push_back({v,u});
                upp[v] = min(upp[v], upp[u]);
            }
            else if(u != parent[v])
                upp[v] = min(upp[v], distance[u]);
        }
    }

    int root;
    vector<int> cut_vertices;
    void dfs_cut_vertice(int v, int d=0)
    {
        visited[v] = true;
        distance[v] = d;
        upp[v] = d;

        for (auto& u : edges[v])
        {
            if (!visited[u])
            {
                parent[u] = v;
                dfs_cut_vertice(u, d+1);

                if(upp[u] >= d && (v != root || edges[v].size() > 1))
                    cut_vertices.push_back(v);
                upp[v] = min(upp[v], upp[u]);
            }
            else if(u != parent[v])
                upp[v] = min(upp[v], distance[u]);
        }
    }

    vector<int> eul_tour;
    void dfs_eul_tour(int v)
    {
        while (edges[v].size())
        {
            int u = edges[v].back();
            edges[v].pop_back();
            edges[u].erase(std::remove(edges[u].begin(), edges[u].end(), v), edges[u].end());
            dfs_eul_tour(u);
        }
        eul_tour.push_back(v);
    }

    void start_dfs(int u)
    {
        visited = vector<bool>(n, false);
        starting_time = finishing_time = vector<int>(n, 0);
        time = 0;
        dfs_sf_time(u);

        visited = vector<bool>(n, false);
        parent = vector<int>(n, -1);
        upp = vector<int>(n, F);
        cut_edges.clear();
        dfs_cut_edge(u);

        visited = vector<bool>(n, false);
        parent = vector<int>(n, -1);
        upp = vector<int>(n, F);
        cut_vertices.clear();
        root = u;
        dfs_cut_vertice(u);

        eul_tour.clear();
        dfs_eul_tour(u);
    }

    
    vector<int> dp, mx;
    // return size of its subtree
    int dfs_centroid(int v, int par=-1)
    {
        dp[v] = 1;
        for (auto& u : edges[v])
            if (u != par)
            {
                int x = dfs_centroid(u, v);
                dp[v] += x;
                mx[v] = max(mx[v], x);
            }
        return dp[v];
    }

    pair<int,int> find_centroid()
    {
        get_input();
        dp = vector<int>(n, 0);
        mx = vector<int>(n, -1);
        dfs_centroid(0);
        int res = F;
        pair<int,int> result;
        result = {-1, -1};

        rep(i, n) mx[i] = max(mx[i], n-dp[i]);
        rep(i, n) res=min(res, mx[i]);
        rep(i, n)
            if (mx[i] == res)
            {
                if (result.first != -1) 
                    result.second = i;
                else 
                    result.first = i;
            }
        
        return result;
    }
};

// Directed Graph
class Graph3
{
public:
    int n, m;
    vector<vector<int>> edges;
    vector<vector<int>> reverse_edges;

    void get_input()
    {
        cin >> n >> m;
        rep(i, m)
        {
            int u, v;
            cin >> u >> v;
            u--, v--;
            edges[u].push_back(v);
            reverse_edges[v].push_back(u);
        }
    }

    vector<bool> visited;
    vector<int> topol;
    void dfs_topol(int v)
    {
        visited[v] = true;
        for (auto& u : edges[v])
            if (!visited[u])
                dfs_topol(u);
	    topol.push_back(v);
    }

    vector<int> find_topological_sort(bool get_input_from_user=false)
    {
        if (get_input_from_user)
            get_input();
        visited = vector<bool>(n, false);
        topol.clear();

        rep(u, n)
            if (!visited[u])
                dfs_topol(u);
        reverse(topol.begin(), topol.end());

        return topol;
    }

    void dfs_scc(int v, vector<vector<int>>& scc, vector<int>& node_scc)
    {
        node_scc[v] = (scc.size()-1);
        scc.back().push_back(v);
        visited[v] = true;
        for (auto& u : reverse_edges[v])
            if (!visited[u])
                dfs_scc(u, scc, node_scc);
    }

    // return { scc index of each vertice, each scc component}
    pair<vector<int>, vector<vector<int>>> scc()
    {
        get_input();
        vector<int> topol_order = find_topological_sort(false);

        vector<vector<int>> scc;
        vector<int> node_scc;
        visited = vector<bool>(n, false);
        for (auto& v : topol_order)
        {
            if(!visited[v])
            {
                scc.push_back(vector<int>());
                dfs_scc(v, scc, node_scc);
            }
        }

        return {node_scc, scc};
    }
};