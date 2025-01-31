#include<bits/stdc++.h> 
using namespace std; 
# define INF 0x3f3f3f3f 
# define RADIUS 5
# define MARGIN 3
# define ONE_RADIAN 0.0174533
# define ARRAY_SIZE 200
// intFPair ==>  Integer Float Pair 
typedef pair<int, float> intFPair; 

// This class is used to store node and give access to them. Also provides methods to use them. 
class Node
{   
    // Node coordinates
    float x = 0;
    float y = 0;
    uint16_t index = 0;

public:
    Node() {};
    // initialize the node
    void init_node(uint16_t index_no, float x_point, float y_point);
    
    // get euclidean distance from this node to node V
    float get_distance(Node V);

    // value getters
    uint16_t get_index() {return this->index;}
    float get_x() {return this->x;}
    float get_y() {return this->y;}

    // projects nodes in different direction according to index
    static const void get_node_coordinates(float obstacle_x, float obstacle_y, uint8_t index, float &node_x, float &node_y);
};

// This class contains basic math funtions needed for this program
class Math
{   
public:
    
    // constructor 
    Math() {}; 
    
    // closest distance between a line connected from start -> end, and a point
    static const float closest_distance(Node start, Node end, Node point);
};
  
// This class represents a directed graph using 
// adjacency list representation 
// It also contains Dijkstra's implementation
class Graph 
{ 
    int V;    // No. of vertices 
  
    // In a weighted graph, we need to store vertex 
    // and weight pair for every edge 
    list< pair<int, float> > *adj; 
  
public: 
    Graph(int V);  // Constructor 
  
    // function to add an edge to graph 
    void addEdge(int u, int v, float w); 
  
    // prints shortest path from s 
    int* shortestPath(int s, int goal, int& navigation_count); 
    
    // True if obstacle is less than radius distance away from line formed between graph_node_start -> graph_node_end
    static const bool intersect(Node obstacle, float radius, Node graph_node_start, Node graph_node_end);
}; 

float const Math::closest_distance(Node start, Node end, Node point)
{
    const float start_x = start.get_x();
    const float start_y = start.get_y();
    const float end_x = end.get_x();
    const float end_y = end.get_y();
    const float point_x = point.get_x();
    const float point_y = point.get_y();

    const float l2 = pow((start_x-end_x),2) + pow((start_y-end_y),2);
    if (l2 < FLT_EPSILON) {
        // start = end
        return point.get_distance(start);
    }

    const float t = (((point_x-start_x) * (end_x-start_x)) + ((point_y-start_y) * (end_y-start_y)))/ l2;
    if (t <= 0) {
        // closest point is start
        return point.get_distance(start);
    } else if (t >= 1) {
        // closest point is end
        return point.get_distance(end);
    } else {
        // closest point is somewhere in the middle
        const float inter_x = start_x + (end_x-start_x)*t;
        const float inter_y = start_y + (end_y-start_y)*t;
        const float dist = sqrtf(pow(inter_x-point_x,2)+pow(inter_y-point_y,2));
        return dist;
    }
    // should never reach here, but just for compilers sake
    return 0.0f;
}
// Allocates memory for adjacency list 
Graph::Graph(int V) 
{ 
    this->V = V; 
    adj = new list<intFPair> [V]; 
} 
  
void Graph::addEdge(int u, int v, float w) 
{ 
    adj[u].push_back(make_pair(v, w)); 
    adj[v].push_back(make_pair(u, w)); 
} 
  
// Prints shortest paths from src to all other vertices 
int* Graph::shortestPath(int src, int goal, int& navigation_count) 
{ 
    // Create a priority queue to store vertices that 
    priority_queue< intFPair, vector <intFPair> , greater<intFPair> > pq; 
  
    // Create a vector for distances and initialize all 
    // distances as infinite (INF) 
    vector<float> dist(V, INF); 
  
    // Insert source itself in priority queue and initialize 
    // its distance as 0. 
    pq.push(make_pair(0, src)); 
    dist[src] = 0; 
    int prevvertex[ARRAY_SIZE];
    static int finalpath[ARRAY_SIZE];

    /* Looping till priority queue becomes empty (or all 
      distances are not finalized) */
    while (!pq.empty()) 
    { 
        // The first vertex in pair is the minimum distance 
        // vertex, extract it from priority queue. 
        // vertex label is stored in second of pair (it 
        // has to be done this way to keep the vertices 
        // sorted distance (distance must be first item 
        // in pair) 
        int u = pq.top().second; 
        pq.pop(); 
  
        // 'i' is used to get all adjacent vertices of a vertex 
        list< pair<int, float> >::iterator i; 
        for (i = adj[u].begin(); i != adj[u].end(); ++i) 
        { 
            // Get vertex label and weight of current adjacent 
            // of u. 
            int v = (*i).first; 
            float weight = (*i).second; 
  
            //  If there is shorted path to v through u. 
            if (dist[v] > dist[u] + weight) 
            { 
                // Updating distance of v 
                dist[v] = dist[u] + weight; 
                pq.push(make_pair(dist[v], v)); 
                prevvertex[v]=u;
            } 
        } 
    } 

    // uncomment to print output
    // Print shortest distances stored in dist[] 
    // printf("Vertex   Distance from Source\n"); 
    // for (int i = 0; i < V; ++i) 
        // printf("%d \t\t %f\n", i, dist[i]); 
    
    int i,k=1;
    for( i=goal;prevvertex[i]!=0;) {
        if(prevvertex[i]!=0)
        {
            finalpath[k]=i;
        }
            
        i=prevvertex[i];  
        k++;
    }
    finalpath[k+1]=src;
    finalpath[k]=i;

    navigation_count = k;
    return finalpath;
} 

void Node:: init_node(uint16_t index_no, float x_point, float y_point)
{
    this-> index = index_no;
    this-> x = x_point;
    this-> y = y_point;
} 

float Node::get_distance(Node V)
{
    const float dist = sqrt(pow((this->x - V.x), 2) + pow((this->y - V.y), 2));
    return dist;
}

// project to 4 corners of every obstacle
const void Node::get_node_coordinates(float obstacle_x, float obstacle_y, uint8_t index, float &node_x, float &node_y)
{
    switch (index) {
        case 0:
            // top right 
            node_x = obstacle_x + cosf(ONE_RADIAN*(45)) * RADIUS + MARGIN;
            node_y = obstacle_y + sinf(ONE_RADIAN*(45)) * RADIUS + MARGIN;
            break;
        case 1:
            // bottom right 
            node_x = obstacle_x + cosf(ONE_RADIAN*(135)) * (RADIUS + MARGIN);
            node_y = obstacle_y + sinf(ONE_RADIAN*(135)) * (RADIUS + MARGIN);
            break;

        case 2:
            // bottom left 
            node_x = obstacle_x + cosf(ONE_RADIAN*(225)) * (RADIUS + MARGIN);
            node_y = obstacle_y + sinf(ONE_RADIAN*(225)) * (RADIUS + MARGIN);
            break;

        case 3:
            // top left
            node_x = obstacle_x + cosf(ONE_RADIAN*(315)) * (RADIUS + MARGIN);
            node_y = obstacle_y + sinf(ONE_RADIAN*(315)) * (RADIUS + MARGIN);
            break;
    }
} 

// check if node start and end reach within "RADIUS" of the obstacle.. if they do, return false
const bool Graph::intersect(Node obstacle, float radius, Node graph_node_start, Node graph_node_end)
{
    const float distance_to_obstacle = Math::closest_distance(graph_node_start, graph_node_end, obstacle);
    if (distance_to_obstacle < radius) {
        // do not allow edge here
        return true;
    }
    return false;
}

// Driver program to test methods of graph class 
int main() 
{      
    // Create new nodes for current location and end location
    Node* robot_location = new Node[2]; // 0 -> current loc, 1 -> Goal
   
    cout << "Enter Robots Current Location \n";
    float start_x = 0;
    float start_y = 0;
    cin >> start_x >> start_y;

    cout << "enter Robots Final Location \n";
    float end_y = 0;
    float end_x = 0;
    cin >> end_x >> end_y;

    uint16_t obstacle_num = 0;
    cout << "Enter the number of Obstacles \n";
    cin >> obstacle_num;

    robot_location[0].init_node(0, start_x, start_y);
    robot_location[1].init_node(4*obstacle_num + 1, end_x, end_y);        
    
    // dynamically create new obstacle nodes 
    Node* obstacle = new Node[obstacle_num];
    Node* obstacle_node = new Node[4 * obstacle_num];

    for(uint8_t i=0; i < obstacle_num; i++) {
        float x = 0;
        float y = 0;
        cout << "Enter location of obstacle \n";
        cin >> x >> y;
        obstacle[i].init_node(i,x,y);
        for (uint8_t j=0; j < 4; j++) {
            float node_x, node_y;
            Node::get_node_coordinates(x, y, j, node_x, node_y);
            // reserve first index for robot start location
            // create 4 obstacle node per obstacle
            obstacle_node[4*i+j].init_node(4*i+j+1, node_x, node_y);
        }
    }
    
    // create the graph given in above figure 
    int V = 4 * obstacle_num + 2; // 2 correspons to start and end of the robot and 4 nodes per obstacle
    Graph g(V);
    const int total_obstacle_nodes = 4*obstacle_num;
    // add valid edges to graph
    for (uint16_t i = 0; i <  total_obstacle_nodes; i++) {
        uint16_t next_obstacle = i +1;
        uint16_t iterator = 0;
        while (iterator != total_obstacle_nodes) {
            iterator ++;
            if (next_obstacle >= total_obstacle_nodes) {
            next_obstacle = 0;
            }
            bool add_edge = true;
            for(uint16_t j=0; j < obstacle_num; j++) {
                if(Graph::intersect(obstacle[j], RADIUS ,obstacle_node[i], obstacle_node[next_obstacle])) {
                    // this edge intersects the obstacle
                    add_edge = false;
                    break;
                }
            }
            if (add_edge) {
                g.addEdge(obstacle_node[i].get_index(), obstacle_node[next_obstacle].get_index(), obstacle_node[i].get_distance(obstacle_node[next_obstacle]));
            }
            next_obstacle ++;
        }
    } 
    
    // add current loc and end loc to graph
    for (uint16_t i = 0; i < total_obstacle_nodes; i++) {
        
        // current location to node edge connections
        bool add_edge = true;
        for(uint16_t j=0; j < obstacle_num; j++) {
            if(Graph::intersect(obstacle[j], RADIUS ,robot_location[0],obstacle_node[i])) {
                // this edge intersects the obstacle
                add_edge = false;
                break;
            }
        }
        if (add_edge) {
            g.addEdge(robot_location[0].get_index(), obstacle_node[i].get_index(), robot_location[0].get_distance(obstacle_node[i]));
        }
        
        // goal to node edge connections
        add_edge = true;
        for(uint16_t j=0; j < obstacle_num; j++) {
            if(Graph::intersect(obstacle[j], RADIUS ,robot_location[1],obstacle_node[i])) {
                // this edge intersects the obstacle
                add_edge = false;
                break;
            }
        }
        if (add_edge) {
            g.addEdge(robot_location[1].get_index(), obstacle_node[i].get_index(), robot_location[1].get_distance(obstacle_node[i]));
        }
    }
    {
        bool add_edge = true;
        for(uint16_t j=0; j < obstacle_num; j++) {
            if(Graph::intersect(obstacle[j], RADIUS ,robot_location[0],robot_location[1])) {
                // this edge intersects the obstacle
                add_edge = false;
                break;
            }
        }
        if (add_edge) {
            g.addEdge(robot_location[0].get_index(), robot_location[1].get_index(), robot_location[1].get_distance(robot_location[0]));
        }
    }
    
    int navigation_count = 0;
    int* vertex_array = g.shortestPath(robot_location[0].get_index(), robot_location[1].get_index(), navigation_count); 

    cout << "safe path is";
    cout << "("<< robot_location[0].get_x() << ", " << robot_location[0].get_y() << ") \n";  
    for (int i=navigation_count; i>1; i--) {
        cout << "("<< obstacle_node[vertex_array[i]].get_x() << ", " << obstacle_node[vertex_array[i]].get_y() << ") \n";
    }
    cout << "("<< robot_location[1].get_x() << ", " << robot_location[1].get_y() << ") \n";  
    return 0; 
} 