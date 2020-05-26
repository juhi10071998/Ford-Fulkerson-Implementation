/*!
 * \author Juhi Mittal (2016B3A70298H)
 * \date 16-04-2020
 * \mainpage  Assignment -2 DAA
 * \Section A Implementation of ford fulkerson algorithm for network flow determination
 * \subsection S Test Case input format
 * 1. Input file should only contain the edges from from vertex v1 to v2 and e as the edge weight between them. \n
 * 2. Example: \n
 * 3. 1 2 2\n.
 * 4. 2 3 3\n
 * 5. 3 4 4\n
 * 6. 4 1 5\n
 */
 

#include <iostream>
#include <limits.h>
#include <string.h>
#include <queue>
#include<vector>
#include<utility>
#include<unordered_map>
#include<bits/stdc++.h>
using namespace std;
/*!
 * \brief This Function Implements the BFS algorithm.
 * \param[in] graph is the vector of vectors of pair to serve as the adjacency list.
 * \param[in] number is the number of nodes.
 * \param[in] s is the source vertex.
 * \param[in] t is the sink vertex.
 * \param[in] parent is the array storing the predecessor of a node in the path obtained
 * \param[out] returns true if the path is presnet between the source and destination.
 * \details This function finds the shortest path from the source to the destination vertex by using the queue data structure. \n
 * The parent array is passed to restore the path found by the BFS algorithm \n 
 * If there exists a path then true is returned else false\n
 */
int bfs(vector<vector <pair<long long,long long> > > graph, long long number,long long s,long long t,long long parent[]){
	vector<long long> visited(number);
	long long i;
	for(i=0;i<visited.size();i++){
		visited[i]=0;
	}
	queue<long long> q;
	q.push(s);
	visited[s]=1;
	parent[s]=-1;
	while(!q.empty()){
		long long u=q.front();
		q.pop();
		for(i=0;i<graph[u].size();i++){
			if(visited[graph[u][i].first]==0){
				visited[graph[u][i].first]=1;
				q.push(graph[u][i].first);
				parent[graph[u][i].first]=u;
			}
		}
	}
	if(visited[t]==1)
	return 1;
	else return 0;
}
/*!
 * \brief This function returns the maximum flow in the graph from the source to the sink vertex
 * \param[in] Graph is a vector of vector representing the nodes and edges in the graph
 * \param[in] s is the source vertex
 * \param[in] t is the sink vertex
 * \param[in] number is the total number of nodes present in the graph
 * \param[out] returns the max flow that can be sent from the source to the sink.
 * \details This function calculates the maximum flow that can be sent iteratively. \n
 * The initial phase copies the input graph into the residual graph, both are updated in subsequent iteartions. \n
 * BFS function is called to find the shortest unexplored path between the sink and the source in the residual graph, if such a path exists, the bottleneck is chosen and updates done. \n 
 * Forward edge in the residual represents the flow which can be ent further while the backward edge the flow that can be diverted back. \n
 * In the original graph the bottleneck value is used to update the max flow value. \n
 * The algorithm terminates when there are no moe paths to be discovered between the source and the sink.
 */
int ford_fulkerson(vector<vector <pair<long long,long long> > > graph,long long s, long long t, long long number){
	/* Initialising the residual graph as the vector of vector of pairs and copying the original graph to this */
	vector<vector <pair<long long,long long> > > rgraph(number);
	long long i,j;
	for(i=0;i<number;i++){
		vector< pair<long long,long long> > rgraph[i];
	}
	for(i=0;i<number;i++){
		for(j=0;j<graph[i].size();j++){
			rgraph[i].push_back(make_pair(graph[i][j].first,graph[i][j].second));
		}
	}
	long long parent[number];
	for(i=0;i<number;i++)
	parent[i]=0;
	long long max_flow=0,u,v;
	vector<long long> vec;
	/* Updating the max flow initialised to zero iteratively on the condition of existence of a path from source to sink in the residual graph*/
while(bfs(rgraph,number,s,t,parent)){
	long long path_flow=99999;
	/* calculate the bottleneck for the path returned by BHS */
	for(v = t; v != s; v = parent[v]) {
			u = parent[v];
			for(i=0;i<rgraph[u].size();i++){
				if(rgraph[u][i].first==v){
					path_flow=min(path_flow,rgraph[u][i].second);
				}
			}
		}
	/* For the path returned update the edges in the residual graph */
	for(v = t; v != s; v = parent[v]){
			u = parent[v];
	/* Update the forward edges in the path by subtracting from each the bottleneck flow and 
	if the edge weight becomes zero, erase the vertex from the graph*/
			for(i=0;i<rgraph[u].size();i++){
				if(rgraph[u][i].first==v)
				rgraph[u][i].second-=path_flow;
				if(rgraph[u][i].second==0)
				rgraph[u].erase(rgraph[u].begin()+i);
			}
	/* To add the backward edge, add the bottleneck flow if the edge alraedy present in the graph, 
	otherwise add the new backward edge with bottleneck flow */
			long long flag=0;
			for(i=0;i<rgraph[v].size();i++){
				if(rgraph[v][i].first==u){
				flag=1;
				rgraph[v][i].second+=path_flow;}
				
			}
			if(!flag){
					rgraph[v].push_back(make_pair(u,path_flow));
				}
		}
	/* add the flow from current path to the max flow value */
	max_flow += path_flow;
}
return max_flow;
}

long long rangeMapper(long long x,unordered_map<long long,long long>& rangeMap,long long& mapper){
	if(rangeMap.find(x)!=rangeMap.end())
		return rangeMap[x];

	rangeMap[x] = mapper;
	mapper+=1;

	return rangeMap[x];
}

int main(){
	
	long long n,e,i,x,y,w,j;
	/* taking input the number of nodes and edges presnet */
	cin >> n >> e;
	vector <vector < pair<long long,long long > > > graph(n);
	unordered_map<long long,long long> rangeMap; 
	long long src,dest;
	long long mapper=0;

	cin >> src >> dest;

	for(i=0;i<n;i++){
		vector< pair<long long,long long> > graph[i];
	}
	/* Taking input as the edge directed from vertex u to vertex w along with the edge weight present.
	and Constructing graph. */
	for(i=0;i<e;i++){
		cin >> x >> y >> w;

		x= rangeMapper(x,rangeMap,mapper);
		y= rangeMapper(y,rangeMap,mapper);
		graph[x].push_back(make_pair(y,w));
	}
	/* taking input the source and sink between which the flow is to be determined */

	src = rangeMapper(src,rangeMap,mapper);
	dest = rangeMapper(dest,rangeMap,mapper);
	
	long long parent[n];
	int flag=bfs(graph,n,src,dest,parent);

	cout << "result is: " << ford_fulkerson(graph,src,dest,n) << endl;

	return 0;
}
/*

8 15
0 7
0 1 10
0 2 5
0 3 15
1 4 9
1 5 15
1 2 4
2 5 8
2 3 4
3 6 16
6 2 6
6 7 10
4 5 15
4 7 10
5 7 10
5 6 15

*/
/*

8 15
1 8
1 2 10
1 3 5
1 4 15
2 5 9
2 6 15
2 3 4
3 6 8
3 4 4
4 7 16
7 3 6
7 8 10
5 6 15
5 8 10
6 8 10
6 7 15

*/
