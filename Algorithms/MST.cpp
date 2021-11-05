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
    int index;
    ll d;
	Edge(int uu, int vv, int dd = 0, int xx = -1) { u = uu; v = vv; d = dd; index = xx; }
};

class DSU
{
public:
    int n;
    vector<int> par;
    DSU(int nn)
    {
        n = nn;
        par = vector<int>(n, -1);
        init();
    }

    int fpar(int x) { return x==par[x] ? x : par[x]=fpar(par[x]); }
    void merge(int u, int v) { par[fpar(u)] = fpar(v);}
    void init(){ rep(i,n) par[i] = i; }
};

class Graph1
{
public:
	int n, m;
	vector<Edge> edges;

    void get_weighted_graph()
	{
		cin >> n >> m;
        rep(i, m)
        {
            int u, v, d;
            cin >> u >> v >> d;
            u--, v--;
            edges.push_back(Edge(u, v, d, i+1));
        }
	}

    // find mst
    // returns {mst weight, index of edges of mst}
    pair<int, vector<int>> kruskal()
    {
        get_weighted_graph();
        DSU dsu(n);
        vector<int> mst;
        ll answer = 0;

        auto cmp_weight = [](Edge a, Edge b) { return (a.d < b.d); };
        sort(edges.begin(), edges.end(), cmp_weight);
        for (auto& e : edges)
        {
            if (dsu.fpar(e.u) == dsu.fpar(e.v))
                continue;
            dsu.merge(e.u, e.v);
            answer += e.d;
            mst.push_back(e.index);
        }
        sort(mst.begin(), mst.end());
        return {answer, mst};
    }

   // Prim: Using Dijkstra from one vertice
};