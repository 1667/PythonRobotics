#include <iostream>
#include <math.h>
#include <map>
#include <vector>
#include <algorithm>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>



using namespace std;
#define map_size 60


class Node {

public:
    int x;
    int y;
    double cost_g;
    double cost_h;
    int pind;
    Node()
    {

    }
    Node(int x, int y, double cost_g,int pind)
    :x(x),y(y),cost_g(cost_g),pind(pind)
    {

    }
    void set_cost_h(double h)
    {
        cost_h = h;
    }
};


typedef pair<int, Node> node_pair;
bool comp_cost(node_pair node1, node_pair node2)
{
    if((node1.second.cost_g+node1.second.cost_h) == (node2.second.cost_g+node2.second.cost_h))
    {
        return node1.second.cost_g < node2.second.cost_g;
    }
    else{
        return (node1.second.cost_g+node1.second.cost_h) < (node2.second.cost_g+node2.second.cost_h);
    }
}

class AStar
{
public:
    AStar()
    {
        make_map();
        // for(int i = 0; i < map_size; i++)
        // {
        //     for(int j = 0; j < map_size; j++)
        //     {
        //         std::cout << obmap[i][j] << " ";
        //     }
        //     std::cout << std::endl;
        // }
        minx = 0;
        miny = 0;
        maxx = map_size;
        maxy = map_size;
        xwidth = map_size;
        ywidth = map_size;

    }
    ~AStar()
    {

    }

    
    int calc_node_index(Node &node)
    {
        return (node.y-miny)*xwidth+(node.x-minx);
    }
    double calc_h(Node &node_s,Node &node_to)
    {
        double d = sqrt(pow((node_s.x-node_to.x),2) + pow((node_s.y-node_to.y),2));
        return d;
    }
    int get_min_f()
    {
        vector<node_pair> vcp;
        vector<node_pair>::iterator v_iter;
        map<int,Node>::iterator iter_map;
        for(iter_map = open_nodes.begin();iter_map != open_nodes.end();iter_map++)
        {
            // std::cout << iter_map->first << " " << iter_map->second.cost_g << " " << iter_map->second.cost_h << std::endl;
            vcp.push_back(node_pair(iter_map->first,iter_map->second));
        }

        sort(vcp.begin(),vcp.end(),comp_cost);
        // for(v_iter=vcp.begin(); v_iter!=vcp.end(); v_iter++) {
		//     cout<<v_iter->first<<" " <<v_iter->second.cost_g << " " <<v_iter->second.cost_h << " " <<  v_iter->second.cost_g+v_iter->second.cost_h << endl;
	    // }
        // // std::cout << vcp.begin()->second.cost_g+vcp.begin()->second.cost_h << " " << vcp.end()->second.cost_g+vcp.end()->second.cost_h << std::endl;
        // std::cout << "================================" << std::endl;
        return vcp.begin()->first;


    }

    bool verify_node(Node& node)
    {
        if(node.x < minx || node.y < miny || node.x > maxx || node.y > maxy)
        {
            return false;
        }
        if(obmap[node.x][node.y] == 1)
        {
            return false;
        }
        return true;
    }

    void a_star_plan(Node &node_s,Node &node_e)
    {
        int index = calc_node_index(node_s);
        open_nodes.insert(make_pair(index,node_s));


        while(true)
        {
            int c_index = get_min_f();
            Node current_node = open_nodes[c_index];

            if(current_node.x == node_e.x && current_node.y == node_e.y)
            {
                printf("find\n");
                goals_pind = current_node.pind;
                node_e.pind = current_node.pind;
                node_e.cost_g = current_node.cost_g;
                std::cout << close_nodes.size() << std::endl;
                break;
            }

            open_nodes.erase(c_index);
            close_nodes.insert(make_pair(c_index,current_node));

            for(int i = 0; i < 8; i++)
            {
                double* tmp_m = motion[i];
                Node tmpNode = Node(current_node.x+tmp_m[0],current_node.y+tmp_m[1],current_node.cost_g+tmp_m[2],c_index);
                tmpNode.set_cost_h(calc_h(tmpNode,node_e));

                if(verify_node(tmpNode) == false)
                {
                    continue;
                }
                int tmp_index = calc_node_index(tmpNode);

                if(close_nodes.find(tmp_index) != close_nodes.end())
                {
                    continue;
                }

                if(open_nodes.find(tmp_index) == open_nodes.end())
                {
                    open_nodes.insert(make_pair(tmp_index,tmpNode));
                }
                else{
                    Node in_node = open_nodes[tmp_index];
                    if(in_node.cost_g > tmpNode.cost_g)
                    {
                        open_nodes[tmp_index] = tmpNode;
                    }
                }

            }
        }
    }
    
    void show_path()
    {
        // map<int,Node>::iterator iter_map;
        // for(iter_map = close_nodes.begin();iter_map != close_nodes.end();iter_map++)
        // {
        //     Node node = iter_map->second;
        //     obmap[node.x][node.y] = 2;
        // }
        int pind = goals_pind;
        while(pind != -1)
        {
            Node node = close_nodes[pind];
            obmap[node.x][node.y] = 2;
            pind = node.pind;

        }
        // for(int i = 0; i < map_size; i++)
        // {
        //     for(int j = 0; j < map_size; j++)
        //     {
        //         std::cout << obmap[i][j] << " ";
        //     }
        //     std::cout << std::endl;
        // }
        cout << " complet " << obmap << " " << obmap[0] <<endl;
        cv::Mat img = Vec2Mat((int *)obmap,map_size,map_size);
        cout << " complet " <<endl;
        cv::namedWindow("map");
        cv::imshow("map",img);
        cv::waitKey(0);
        cv::imwrite("a.jpg",img);

    }

    cv::Mat Vec2Mat(int *array, int row, int col)
    {
    
        // int *tmpa = (int *)array[2];
        // cout << "create image " << array << " " << array[0] <<endl;
        cv::Mat img(row ,col,  CV_8UC1);


        uchar *ptmp = NULL;

        // for(int i = 0; i < map_size; i++)
        // {
        //     for(int j = 0; j < map_size; j++)
        //     {
        //         std::cout << array[i*map_size+j] << " ";
        //     }
        //     std::cout << std::endl;
        // }

        for (int i = 0; i <row; ++i)
        {
            ptmp = img.ptr<uchar>(i);
            for (int j = 0; j < col; ++j)
            {

                ptmp[j] = (uchar)array[i*map_size+j];
                // ptmp[j] = obmap[i][j];
                if(ptmp[j] == 1)
                {
                    ptmp[j] = 255;
                }
                else if(ptmp[j] == 2)
                {
                    ptmp[j] = 128;
                }
                // printf("%d",ptmp[j]);
            }
        }
        
        return img;
    
    }


    void make_map()
    {
        for(int i = 0; i < map_size; i++)
        {
            for(int j = 0; j < map_size; j++)
            {
                if(i == 0 || i == map_size-1 || j == 0 || j == map_size-1)
                {
                    obmap[i][j] = 1;
                }
                if(i >= 20 && j == 20)
                {
                    obmap[i][j] = 1;
                }
                if(i <= 40 && j == 40)
                {
                    obmap[i][j] = 1;
                }

            }
        }
    }

private:
    double motion[8][3] = {
            {1,0,1},
            {0,1,1},
            {-1,0,1},
            {0,-1,1},
            {-1,-1,sqrt(2)},
            {-1,1,sqrt(2)},
            {1,-1,sqrt(2)},
            {1,1,sqrt(2)}
        };

    map<int,Node> open_nodes;
    map<int,Node> close_nodes;

    int minx = 0;
    int miny = 0;
    int xwidth = 0;
    int ywidth = 0;
    int maxx = 0;
    int maxy = 0;
    int obmap[map_size][map_size] = {0};
    int goals_pind = -1;
    
};

int main(int argc, char**argv){
    std::cout << "hello" << std::endl;

    AStar star = AStar();
    Node Node_s = Node(30,10,0,-1);
    Node Node_e = Node(10,50,0,-1);
    Node_s.set_cost_h(star.calc_h(Node_s,Node_e));
    star.a_star_plan(Node_s,Node_e);
    star.show_path();
    // for(int i = 0; i < map_size; i++)
    // {
    //     for(int j = 0; j < map_size; j++)
    //     {
    //         std::cout << obmap[i][j] << " ";
    //     }
    //     std::cout << std::endl;
    // }



}