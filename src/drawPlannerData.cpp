#include "drawPlannerData.h"
//Just for debugging
//#include<iostream>
//#include<thread>




void getXY(const ob::PlannerDataVertex &v, double &x, double &y)
{
    const auto* state = v.getState();
    const auto* state2D = state->as<ob::RealVectorStateSpace::StateType>();
    x = state2D->values[0];
    y = state2D->values[1];
}


//Extracts graph data for plotting from the planner data and the solution path. Returns graph data
graphData extractData(const ob::PlannerData &data, const og::PathGeometric *spath)
{
    std::vector<unsigned int> edges;
    double x0, x1, y0, y1, xi, yi;
    std::vector<double> X = {0, 0};
    std::vector<double> Y = {0, 0};

    std::vector<double> sX;
    std::vector<double> sY;
    std::vector<edge> tree;

    


    //Plot tree
    for (unsigned int i = 0; i < data.numVertices(); ++i)
    {   
        getXY(data.getVertex(i), X[0], Y[0]);
        //Get state information
        data.getEdges(i,edges);
        for (auto oi:edges)
        {
            getXY(data.getVertex(oi), X[1], Y[1]);
            tree.push_back(edge(X,Y));
        }
    }


    if(spath != nullptr)
    {
        for (unsigned int i = 0; i < spath->getStateCount(); ++i)
        {
            getXY(spath->getState(i),xi,yi);
            sX.push_back(xi);
            sY.push_back(yi);
        }
    }

    return graphData(tree,sX,sY);;
}

//Plots the extracted graph data
void drawGraphData(graphData data){
    //std::cout<< "\n1.function called - " << std::this_thread::get_id() << std::endl; //For debug - displays thread id.
    static std::mutex mutex;
    std::lock_guard<std::mutex> lock(mutex); // This locks the mutex, and automatically unlocks it after the function goes out of scope.(After plot is closed)
    //std::cout<< "\n2.mutex locked - " << std::this_thread::get_id() << std::endl;
    //Plot Edges
    for (auto e:data.tree){
        plt::plot(e.X, e.Y, "-b");
    }
    //Plot solution path
    if (data.sX.size()>0){
        plt::plot(data.sX,data.sY,{{"color", "red"}, {"linestyle", "-"}, {"linewidth", "2.5"}});
    }
    //std::cout<< "\n3.Displaying Plot - " << std::this_thread::get_id() << std::endl;
    plt::show();
    //std::cout<< "\n4.Finishing execution - " << std::this_thread::get_id() << std::endl;
}

//Directly plots graph data. Slow, but single step.
void drawGraph(const ob::PlannerData &data, const og::PathGeometric *spath)
{
    std::vector<unsigned int> edges;
    double x0, x1, y0, y1, xi, yi;
    std::vector<double> X = {0, 0};
    std::vector<double> Y = {0, 0};
    std::vector<double> sX;
    std::vector<double> sY;
    auto n = data.numVertices();

    //Plot tree
    for (unsigned int i = 0; i < data.numVertices(); ++i)
    {   
        getXY(data.getVertex(i), X[0], Y[0]);
        //Get state information
        data.getEdges(i,edges);
        for (auto oi:edges)
        {
            getXY(data.getVertex(oi), X[1], Y[1]);
            plt::plot(X,Y,"-b");
        }
    }


    if(spath != nullptr)
    {
        for (unsigned int i = 0; i < spath->getStateCount(); ++i)
        {
            getXY(spath->getState(i),xi,yi);
            sX.push_back(xi);
            sY.push_back(yi);
        }
        plt::plot(sX,sY,{{"color", "red"}, {"linestyle", "-"}, {"linewidth", "2.5"}});
    }


    plt::show();
}