// TAPLite.cpp : This file contains the 'main' function. Program execution begins and ends there.

#define FLOAT_ACCURACY 1.0E-15
#define NO_COSTPARAMETERS 4
#define IVTT 0
#define OVTT 1
#define MONETARY 2
#define DIST 3

#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define SQR(x) ((x) * (x))
#define FABS(x) ((x) >= 0 ? (x) : (-x))

#define INVALID -1

#define VALID(x) ((x) != -1)

#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <omp.h>

#define MAX_MODE_TYPES  10

//#ifndef _win32
//void fopen_s(FILE** file, const char* fileName, const char* mode)
//{
//    *file = fopen(fileName, mode);
//}
//#endif

typedef double cost_vector[NO_COSTPARAMETERS];

struct link_record {
    int internal_from_node_id;
    int internal_to_node_id;
    int link_id;
    int external_from_node_id;
    int external_to_node_id;

    double Lane_Capacity; 
    double Link_Capacity;
    double lanes; 
    double FreeTravelTime;
    double free_speed; 
    double Cutoff_Speed; 
        

    double VDF_Alpha;
    double BoverC;
    double VDF_Beta;
    double VDF_plf;
    double Q_cd, Q_n, Q_cp, Q_s;

    double length;
    double Speed;
    std::string allowed_uses;
    int mode_allowed_use[MAX_MODE_TYPES];
    double mode_MainVolume[MAX_MODE_TYPES];
    double mode_SubVolume[MAX_MODE_TYPES];  
    double mode_Toll[MAX_MODE_TYPES];
    double mode_AdditionalCost[MAX_MODE_TYPES];

    double Travel_time;
    double GenCost;
    double GenCostDer;
    double Ref_volume;
    std::string geometry; 
    link_record()
    {
        VDF_Alpha = 0.15;
        VDF_Beta = 4;
        VDF_plf = 1; 

        Q_cd = 1.0;
        Q_n = 1.0; 
        Q_cp = 0.28125 /*0.15*15/8*/; 
        Q_s = 4; 
    
    }
};

struct mode_type {
    std::string mode_type;
    float vot;
    float pce;
    float occ;
    std::string demand_file;
};


mode_type g_mode_type_vector[MAX_MODE_TYPES];


struct list_item {
    struct list_item* next_item;
    int value;
};

typedef struct list_item* sorted_list;

struct lex_node {
    int value;
    struct lex_node* next_alternative;
    struct lex_node* next_item;
};

class CDTACSVParser {
public:
    char Delimiter;
    bool IsFirstLineHeader;
    // for DataHub CSV files
    bool m_bSkipFirstLine;
    bool m_bDataHubSingleCSVFile;
    bool m_bLastSectionRead;

    std::ifstream inFile;
    std::string mFileName;
    std::string m_DataHubSectionName;
    std::string SectionName;

    std::vector<std::string> LineFieldsValue;
    std::vector<int> LineIntegerVector;
    std::vector<std::string> Headers;
    std::map<std::string, int> FieldsIndices;

    CDTACSVParser()
        : Delimiter{ ',' },
        IsFirstLineHeader{ true },
        m_bSkipFirstLine{ false },
        m_bDataHubSingleCSVFile{ false },
        m_bLastSectionRead{ false }
    {
    }

    ~CDTACSVParser()
    {
        if (inFile.is_open())
            inFile.close();
    }

    // inline member functions
    std::vector<std::string> GetHeaderVector() { return Headers; }
    void CloseCSVFile() { inFile.close(); }

    void ConvertLineStringValueToIntegers();
    bool OpenCSVFile(std::string fileName, bool b_required);
    bool ReadRecord();
    bool ReadSectionHeader(std::string s);
    bool ReadRecord_Section();
    std::vector<std::string> ParseLine(std::string line);
    bool GetValueByFieldName(std::string field_name,
        std::string& value,
        bool required_field = true);
    template <class T>
    bool GetValueByFieldName(std::string field_name,
        T& value,
        bool required_field = true,
        bool NonnegativeFlag = true);
    template <class T>
    bool GetValueByKeyName(std::string field_name,
        T& value,
        bool required_field = true,
        bool NonnegativeFlag = true);
};

// definitions of CDTACSVParser member functions
void CDTACSVParser::ConvertLineStringValueToIntegers()
{
    LineIntegerVector.clear();
    for (unsigned i = 0; i < LineFieldsValue.size(); ++i)
    {
        std::string si = LineFieldsValue[i];
        int value = atoi(si.c_str());

        if (value >= 1)
            LineIntegerVector.push_back(value);
    }
}

bool CDTACSVParser::OpenCSVFile(std::string fileName, bool b_required)
{
    mFileName = fileName;
    inFile.open(fileName.c_str());

    if (inFile.is_open())
    {
        if (IsFirstLineHeader)
        {
            std::string s;
            std::getline(inFile, s);
            std::vector<std::string> FieldNames = ParseLine(s);

            for (size_t i = 0; i < FieldNames.size(); i++)
            {
                std::string tmp_str = FieldNames.at(i);
                size_t start = tmp_str.find_first_not_of(" ");

                std::string name;
                if (start == std::string::npos)
                {
                    name = "";
                }
                else
                {
                    name = tmp_str.substr(start);
                    // TRACE("%s,", name.c_str());
                }
                FieldsIndices[name] = (int)i;
            }
        }
        return true;
    }
    else
    {
        if (b_required)
        {
            // g_program_stop();
        }
        return false;
    }
}

bool CDTACSVParser::ReadRecord()
{
    LineFieldsValue.clear();

    if (inFile.is_open())
    {
        std::string s;
        std::getline(inFile, s);
        if (s.length() > 0)
        {
            LineFieldsValue = ParseLine(s);
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }
}

std::vector<std::string> CDTACSVParser::ParseLine(std::string line)
{
    std::vector<std::string> SeperatedStrings;
    std::string subStr;

    if (line.length() == 0)
        return SeperatedStrings;

    std::istringstream ss(line);

    if (line.find_first_of('"') == std::string::npos)
    {
        while (std::getline(ss, subStr, Delimiter))
        {
            SeperatedStrings.push_back(subStr);
        }

        if (line.at(line.length() - 1) == ',')
        {
            SeperatedStrings.push_back("");
        }
    }
    else
    {
        while (line.length() > 0)
        {
            size_t n1 = line.find_first_of(',');
            size_t n2 = line.find_first_of('"');

            if (n1 == std::string::npos &&
                n2 == std::string::npos)  // last field without double quotes
            {
                subStr = line;
                SeperatedStrings.push_back(subStr);
                break;
            }

            if (n1 == std::string::npos && n2 != std::string::npos)  // last field with double
                // quotes
            {
                size_t n3 = line.find_first_of('"', n2 + 1);  // second double quote

                // extract content from double quotes
                subStr = line.substr(n2 + 1, n3 - n2 - 1);
                SeperatedStrings.push_back(subStr);

                break;
            }

            if (n1 != std::string::npos && (n1 < n2 || n2 == std::string::npos))
            {
                subStr = line.substr(0, n1);
                SeperatedStrings.push_back(subStr);
                if (n1 < line.length() - 1)
                {
                    line = line.substr(n1 + 1);
                }
                else  // comma is the last char in the line string, push an empty string to the back
                      // of vector
                {
                    SeperatedStrings.push_back("");
                    break;
                }
            }

            if (n1 != std::string::npos && n2 != std::string::npos && n2 < n1)
            {
                size_t n3 = line.find_first_of('"', n2 + 1);  // second double quote
                subStr = line.substr(n2 + 1, n3 - n2 - 1);
                SeperatedStrings.push_back(subStr);
                size_t idx = line.find_first_of(',', n3 + 1);

                if (idx != std::string::npos)
                {
                    line = line.substr(idx + 1);
                }
                else
                {
                    break;
                }
            }
        }
    }
    return SeperatedStrings;
}

bool CDTACSVParser::GetValueByFieldName(std::string field_name,
    std::string& value,
    bool required_field)
{
    if (FieldsIndices.find(field_name) == FieldsIndices.end())
    {
        if (required_field)
        {
            // dtalog.output() << "[ERROR] Field " << field_name << " in file " << mFileName << "
            // does not exist. Please check the file." << '\n'; g_DTA_log_file << "[ERROR] Field "
            // << field_name << " in file " << mFileName << " does not exist. Please check the
            // file." << '\n'; g_program_stop();
        }
        return false;
    }
    else
    {
        if (LineFieldsValue.size() == 0)
        {
            return false;
        }

        unsigned int index = FieldsIndices[field_name];
        if (index >= LineFieldsValue.size())
        {
            return false;
        }
        std::string str_value = LineFieldsValue[index];

        if (str_value.length() <= 0)
        {
            return false;
        }

        value = str_value;
        return true;
    }
}

// Peiheng, 03/22/21, to avoid implicit instantiations in flash_dta.cpp and main_api.cpp for this
// template function only all the other non-inline functions are implemented in utils.cpp
template <class T>
bool CDTACSVParser::GetValueByFieldName(std::string field_name,
    T& value,
    bool required_field,
    bool NonnegativeFlag)
{
    if (FieldsIndices.find(field_name) == FieldsIndices.end())
    {
        if (required_field)
        {
            // dtalog.output() << "[ERROR] Field " << field_name << " in file " << mFileName.c_str()
            // << " does not exist. Please check the file." << '\n'; g_DTA_log_file << "[ERROR]
            // Field " << field_name << " in file " << mFileName.c_str() << " does not exist. Please
            // check the file." << '\n'; g_program_stop();
        }
        return false;
    }
    else
    {
        if (LineFieldsValue.size() == 0)
        {
            return false;
        }

        int size = (int)(LineFieldsValue.size());
        if (FieldsIndices[field_name] >= size)
        {
            return false;
        }

        std::string str_value = LineFieldsValue[FieldsIndices[field_name]];

        if (str_value.length() <= 0)
        {
            return false;
        }

        std::istringstream ss(str_value);

        T converted_value;
        ss >> converted_value;

        if (/*!ss.eof() || */ ss.fail())
        {
            return false;
        }

        // if (required_field)
        //{
        //     if(NonnegativeFlag)
        //     {
        //         if (converted_value < 0)
        //             converted_value = 0;
        //     }
        // }

        value = converted_value;
        return true;
    }
}

std::map<int, int> g_map_external_node_id_2_node_seq_no;
std::map<int, int> g_map_node_seq_no_2_external_node_id;

int ListFind(int Value, sorted_list list);
void ListAdd(int value, sorted_list* list);
int ListRemove(int value, sorted_list* list);
void ListFree(sorted_list list);
sorted_list ListCopy(sorted_list list);
void ListMerge(sorted_list list1, sorted_list* list2);
sorted_list ListIntersect(sorted_list list1, sorted_list list2);
sorted_list ListDifference(sorted_list list1, sorted_list list2);
int ListsAreEqual(sorted_list list1, sorted_list list2);
int ListSize(sorted_list list);

struct link_record* Link;
int* FirstLinkFrom;
int* LastLinkFrom;
sorted_list* LinksTo;

double Link_GenCost(int k, double* Volume);
double LinkCost_Integral(int k, double* Volume);
double Link_GenCostDer(int k, double* Volume);

void* Alloc_1D(int dim1, size_t size);
void** Alloc_2D(int dim1, int dim2, size_t size);
void Free_2D(void** Array, int dim1, int dim2);
void*** Alloc_3D(int dim1, int dim2, int dim3, size_t size);
void Free_3D(void*** Array, int dim1, int dim2, int dim3);

/* LINK VECTOR FUNCTIONS */
void ClearVolume(double* VolumeArray);
void VolumeDifference(double* Volume1, double* Volume2, double* Difference);
void UpdateVolume(double* MainVolume, double* SubVolume, double Lambda);

/* LINK COST FUNCTIONS */
void UpdateLinkAdditionalCost(void);
double UpdateLinkCost(double* Volume);
void UpdateLinkCostDer(double* Volume);
void GetLinkTravelTimes(double* Volume, double* TravelTime);
double TotalLinkCost(double* Volume);

/*  LINK OBJECTIVE FUNCTION */
double OF_Links(double* MainVolume);
double OF_LinksDirectionalDerivative(double* MainVolume, double* SDVolume, double Lambda);

void InitLinks(void);
void CloseLinks(void);

void Read_ODflow(double* TotalODflow, int* no_modes, int* no_zones);

int Minpath(int m, int Orig, int* PredLink, double* Cost_to);
double FindMinCostRoutes(int** MinPathPredLink);
void Assign(double** ODflow, int** MinPathPredLink, double* Volume);

void InitLineSearch(void);
void CloseLineSearch(void);
double LinksSDLineSearch(double* MainVolume, double* SDVolume);

void print_ls_header(FILE* fp);
void ls_report(FILE* fp);

void StatusMessage(const char* group, const char* format, ...);
void ExitMessage(const char* format, ...);


/* Gloabal variables */

int no_zones, no_modes, no_nodes, no_links, FirstThruNode;
int AssignIterations = 20;
int demand_period_starting_hours = 7;
int	demand_period_ending_hours = 8;


double** ODflow, TotalODflow;


double*** MDODflow;
double*** MDRouteCost;
/* Local Declarations */
/* void FW(void); Should there be a function for fw, or should it be included in main? */
static void Init(int input_no_modes,int input_no_zones);
static void Close();
static void InitODflow(int input_no_modes,int input_no_zones);
static void CloseODflow(void);
/* End of local declarations. */

FILE* logfile;
int shortest_path_log_flag = 0;

FILE* link_performance_file;

#define INVALID -1      /* Represents an invalid value. */
#define BIGM 9999999      /* Represents an invalid value. */
#define WAS_IN_QUEUE -7 /* Shows that the node was in the queue before. (7 is for luck.) */

double Link_QueueVDF(int k, double Volume, double& IncomingDemand, double &DOC, double& P, double& t0, double& t2, double& t3, double& vt2, double& Q_mu, double& Q_gamma, double & congestion_ref_speed,
    double& avg_queue_speed, double& avg_QVDF_period_speed, double& Severe_Congestion_P, double model_speed[300]);

int Minpath(int mode, int Orig, int* PredLink, double* CostTo)
{
    int node, now, NewNode, k, Return2Q_Count = 0;
    double NewCost;
    int* QueueNext;
    int QueueFirst, QueueLast;

    QueueNext = (int*)Alloc_1D(no_nodes, sizeof(int));

    for (node = 1; node <= no_nodes; node++)
    {
        QueueNext[node] = INVALID;
        CostTo[node] = BIGM;
        PredLink[node] = INVALID;
    }

    now = Orig;
    QueueNext[now] = WAS_IN_QUEUE;
    PredLink[now] = INVALID;
    CostTo[now] = 0.0;

    QueueFirst = QueueLast = INVALID;

    while ((now != INVALID) && (now != WAS_IN_QUEUE))
    {
        if (now >= FirstThruNode || now == Orig)  // this is the key implementation for FirstThruNode on connector
        {
            for (k = FirstLinkFrom[now]; k <= LastLinkFrom[now]; k++)
            {

                if (Link[k].mode_allowed_use[mode] == 0)  // implementation for allowed uses 
                    continue; 
                /* For every link that terminate at "now": */

                NewNode = Link[k].internal_to_node_id;
                NewCost = CostTo[now] + Link[k].Travel_time + Link[k].mode_AdditionalCost[mode];

                if (CostTo[NewNode] > NewCost)
                {
                    /* If the new label is better than the old one, correct it, and make sure that
                     * the new node to the queue. */

                    CostTo[NewNode] = NewCost;
                    PredLink[NewNode] = k;

                    /* If the new node was in the queue before, add it as the first in the queue. */
                    if (QueueNext[NewNode] == WAS_IN_QUEUE)
                    {
                        QueueNext[NewNode] = QueueFirst;
                        QueueFirst = NewNode;
                        if (QueueLast == INVALID)
                            QueueLast = NewNode;
                        Return2Q_Count++;
                    }

                    /* If the new node is not in the queue, and wasn't there before, add it at the
                     * end of the queue. */
                    else if (QueueNext[NewNode] == INVALID && NewNode != QueueLast)
                    {
                        if (QueueLast != INVALID)
                        { /*Usually*/
                            QueueNext[QueueLast] = NewNode;
                            QueueLast = NewNode;
                        }
                        else
                        { /* If the queue is empty, initialize it. */
                            QueueFirst = QueueLast = NewNode;
                            QueueNext[QueueLast] = INVALID;
                        }
                    }

                    /* If the new node is in the queue, just leave it there. (Do nothing) */
                }
            }
        }

        /* Get the first node out of the queue, and use it as the current node. */
        now = QueueFirst;
        if ((now == INVALID) || (now == WAS_IN_QUEUE))
            break;

        QueueFirst = QueueNext[now];
        QueueNext[now] = WAS_IN_QUEUE;
        if (QueueLast == now)
            QueueLast = INVALID;
    }

    free(QueueNext);

    return (Return2Q_Count);
}

/* Find minimum cost routes .
Input: 	None
Output:	RouteCost - route generalized cost, by origin and destination
        MinPathSuccLink - trees of minimum cost routes, by destination and node. */

double FindMinCostRoutes(int*** MinPathPredLink)
{

    double** CostTo;

    CostTo = (double**)Alloc_2D(no_zones, no_nodes, sizeof(double));
    StatusMessage("Minpath", "Starting the minpath calculations.");
    double* system_least_travel_time_org_zone = (double*)Alloc_1D(no_zones, sizeof(double));

#pragma omp for
    for (int Orig = 1; Orig <= no_zones; Orig++)
    {
        for(int m = 1; m <= no_modes; m++)
        {
        system_least_travel_time_org_zone[Orig] = 0;

        Minpath(m, Orig, MinPathPredLink[m][Orig], CostTo[Orig]);
        if (MDRouteCost != NULL)
        {
            for (int Dest = 1; Dest <= no_zones; Dest++)
            {
                 MDRouteCost[m][Orig][Dest] = BIGM; // initialization 

                if (CostTo[Orig][Dest] < 0.0)
                    ExitMessage("Negative cost %lg from Origin %d to Destination %d.",
                        (double)CostTo[Orig][Dest], Orig, Dest);

                if (CostTo[Orig][Dest] <= BIGM - 1)  // feasible cost 
                {
                    MDRouteCost[m][Orig][Dest] = CostTo[Orig][Dest];
                    system_least_travel_time_org_zone[Orig] += MDRouteCost[m][Orig][Dest] * MDODflow[m][Orig][Dest];
                }
                else
                {
                    int debug_flag = 1; 
                }
            }
        }
        }
    }

    double system_least_travel_time = 0;
    for (int Orig = 1; Orig <= no_zones; Orig++)
    {
        system_least_travel_time+=system_least_travel_time_org_zone[Orig];
    }
    

    // free(CostTo);
    Free_2D((void**)CostTo, no_zones, no_nodes);
    free(system_least_travel_time_org_zone);
    StatusMessage("Minpath", "Found all minpath.");
    return system_least_travel_time;
}

/* Assign OD flows to links according to the routes in MinPathPredLink. */

void Assign(int Assignment_iteration_no, double*** ODflow, int*** MinPathPredLink, double* Volume)
{
    int Dest, Orig, k;
    int CurrentNode;
    double RouteFlow;

    //// Create a log file
    // FILE* logfile_od;
    // fopen(&logfile_od, "assignment_logfile_od.txt", "a+");
    // if (logfile_od == NULL) {
    //	printf("Error opening file!\n");
    //	return;
    // }

    for (k = 1; k <= no_links; k++)
    {
        Volume[k] = 0.0;

        for (int m = 1; m <= no_modes; m++)
            Link[k].mode_SubVolume[m] = 0.0;
    }
    //StatusMessage("Assign", "Starting assign.");
    for(int m = 1; m <= no_modes; m++)
    { 
    for (Orig = 1; Orig <= no_zones; Orig++)
    {
        //	printf("Assign", "Assigning origin %6d.", Orig);
        for (Dest = 1; Dest <= no_zones; Dest++)
        {
            if (Dest == Orig)
                continue;

            RouteFlow = ODflow[m][Orig][Dest];  //test
            if (RouteFlow == 0)
                continue;

            if (MDRouteCost[m][Orig][Dest] >= BIGM - 1)  // if feasible 
                continue; 

            CurrentNode = Dest;
            //double total_travel_time = 0;
            //double total_length = 0;
            //double total_FFTT = 0;

            while (CurrentNode != Orig)
            {
                k = MinPathPredLink[m][Orig][CurrentNode];
                if (k == INVALID)
                {
                    printf("A problem in mincostroutes.c (Assign): Invalid pred for node %d Orig%d \n\n", CurrentNode, Orig);
                    break;
                }
                Volume[k] += RouteFlow * g_mode_type_vector[m].pce;
                Link[k].mode_SubVolume[m]+= RouteFlow;  // pure volume 

                CurrentNode = Link[k].internal_from_node_id;

                //total_travel_time += Link[k].Travel_time;
                //total_length += Link[k].length;
                //total_FFTT += Link[k].FreeTravelTime;
                // Log the link and OD pair

                //if (shortest_path_log_flag)
                //{
                //    fprintf(logfile,
                //        "Iteration: %d, Origin: %d, Destination: %d, Route Flow: %f,Link: %d, "
                //        "from: %d, to: %d, tt: %f, FFTT: %f, delay: %f, length: %f \n",
                //        Assignment_iteration_no, Orig, Dest, RouteFlow, k,
                //        Link[k].external_from_node_id, Link[k].external_to_node_id,
                //        total_travel_time, total_FFTT, total_travel_time - total_FFTT,
                //        total_length);
                //}
            }

            // fprintf(logfile_od, "Origin: %d, Destination: %d, Route Flow: %f, tt: %f, length:
            // %f,FFTT:%f \n", Orig, Dest, RouteFlow, 	total_travel_time, total_length,
            //total_FFTT);
        }
    }
    }

    /*	fclose(logfile_od)*/;
    StatusMessage("Assign", "Finished assign.");
}

int get_number_of_nodes_from_node_file(int& number_of_zones, int& l_FirstThruNode)
{
    number_of_zones = 0;
    CDTACSVParser parser_node;
    l_FirstThruNode = 1;
    int number_of_nodes = 0;

    if (parser_node.OpenCSVFile("node.csv", true))
    {
        while (parser_node.ReadRecord())  // if this line contains [] mark, then we will also read
            // field headers.
        {
            // Read node id
            int node_id = 0;
            int zone_id = 0;
            parser_node.GetValueByFieldName("node_id", node_id);
            parser_node.GetValueByFieldName("zone_id", zone_id);

            g_map_node_seq_no_2_external_node_id[number_of_nodes + 1] = node_id;
            g_map_external_node_id_2_node_seq_no[node_id] =
                number_of_nodes + 1;  // this code node sequential number starts from 1

            if (zone_id >= 1 && zone_id > number_of_zones)
                number_of_zones = zone_id;

            if (zone_id == 0 && l_FirstThruNode == 1 /* not initialized*/)
                l_FirstThruNode = number_of_nodes + 1;  //use sequential node id

            number_of_nodes++;
        }

        parser_node.CloseCSVFile();
    }

    return number_of_nodes;
}

int get_number_of_links_from_link_file()
{
    CDTACSVParser parser_link;

    int number_of_links = 0;

    if (parser_link.OpenCSVFile("link.csv", true))
    {
        while (parser_link.ReadRecord())  // if this line contains [] mark, then we will also read
            // field headers.
        {
            int link_id = 0;
            int internal_from_node_id = 0;
            // Read link_id
            parser_link.GetValueByFieldName("link_id", link_id);
            parser_link.GetValueByFieldName("from_node_id", internal_from_node_id);

            number_of_links++;
        }

        parser_link.CloseCSVFile();
    }

    return number_of_links;
}


void read_settings_file()
{

    CDTACSVParser parser_settings;

    if (parser_settings.OpenCSVFile("settings.csv", true))
    {
        while (parser_settings.ReadRecord())  // if this line contains [] mark, then we will also read
            // field headers.
        {

            parser_settings.GetValueByFieldName("number_of_iterations", AssignIterations);
            parser_settings.GetValueByFieldName("demand_period_starting_hours", demand_period_starting_hours);
            parser_settings.GetValueByFieldName("demand_period_ending_hours", demand_period_ending_hours);
         	

        }

        parser_settings.CloseCSVFile();
    }

    return;
}


void read_mode_type_file()
{

    CDTACSVParser parser_mode_type;

    if (parser_mode_type.OpenCSVFile("mode_type.csv", true))
    {

        no_modes = 0;
        while (parser_mode_type.ReadRecord())  // if this line contains [] mark, then we will also read
            // field headers.
        {
            no_modes += 1;
            if(no_modes < MAX_MODE_TYPES)
            {
            parser_mode_type.GetValueByFieldName("mode_type", g_mode_type_vector[no_modes].mode_type);
            parser_mode_type.GetValueByFieldName("vot", g_mode_type_vector[no_modes].vot);
            parser_mode_type.GetValueByFieldName("pce", g_mode_type_vector[no_modes].pce);
            parser_mode_type.GetValueByFieldName("occ", g_mode_type_vector[no_modes].occ);
            parser_mode_type.GetValueByFieldName("demand_file", g_mode_type_vector[no_modes].demand_file);
            }

        }

        parser_mode_type.CloseCSVFile();
    }

    if (no_modes == 0)  //default 
    {
        g_mode_type_vector[1].demand_file = "demand.csv";
        g_mode_type_vector[1].mode_type = "auto";
        g_mode_type_vector[1].vot = 10;
        g_mode_type_vector[1].pce = 1;
        g_mode_type_vector[1].occ = 1;
        no_modes = 1;
    }
    return;
}

int main(int argc, char** argv)
{
    double* MainVolume, * SubVolume, * SDVolume, Lambda;
    int*** MDMinPathPredLink;

    read_settings_file();
    read_mode_type_file();

    no_nodes = get_number_of_nodes_from_node_file(no_zones, FirstThruNode);
    no_links = get_number_of_links_from_link_file();

    printf("no_nodes= %d, no_zones = %d, FirstThruNode (seq No) = %d, no_links = %d\n", no_nodes, no_zones,
        FirstThruNode, no_links);

    fopen_s(&link_performance_file, "link_performance.csv", "w");
    if (link_performance_file == NULL)
    {
        printf("Error opening file!\n");
        return 1;
    }
    fclose(link_performance_file);

    fopen_s(&link_performance_file, "link_performance.csv", "a+");
    fopen_s(&logfile, "TAP_log.csv", "w");  // Open the log file for writing.

    double system_wide_travel_time = 0;
    double system_least_travel_time = 0;

    //fprintf(logfile,
    //    "iteration_no,link_id,internal_from_node_id,internal_to_node_id,volume,capacity,voc,"
    //    "fftt,travel_time,delay\n");



    Init(no_modes, no_zones);
    int iteration_no = 0;
    MainVolume = (double*)Alloc_1D(no_links, sizeof(double));
    SDVolume = SubVolume = (double*)Alloc_1D(
        no_links, sizeof(double)); /* Compute search direction and sub-volume in the same place. */
    MDMinPathPredLink = (int***)Alloc_3D(no_modes,no_zones, no_nodes, sizeof(int));

    // InitFWstatus(&fw_status);
    system_wide_travel_time = UpdateLinkCost(MainVolume);  // set up the cost first using FFTT

    fprintf(link_performance_file,
        "iteration_no,link_id,internal_from_node_id,internal_to_node_id,volume,ref_volume,"
        "capacity,D,doc,fftt,travel_time,VDF_alpha,VDF_beta,VDF_plf,speed,VMT,VHT,PMT,PHT,geometry,");

    for (int m = 1; m <= no_modes; m++)
        fprintf(link_performance_file, "mod_vol_%s,", g_mode_type_vector[m].mode_type.c_str());

    fprintf(link_performance_file, "P,t0,t2,t3,vt2,mu,Q_gamma,free_speed,cutoff_speed,congestion_ref_speed,avg_queue_speed,avg_QVDF_period_speed,Severe_Congestion_P,");

    for (int t = demand_period_starting_hours * 60; t < demand_period_ending_hours * 60; t += 5)
    {
            int hour = t / 60;
            int minute = t - hour * 60;

            fprintf(link_performance_file, "spd_%02d:%02d,", hour, minute);
    }

    fprintf(link_performance_file, "\n"); 

    system_least_travel_time = FindMinCostRoutes(MDMinPathPredLink);
    Assign(iteration_no, MDODflow, MDMinPathPredLink, MainVolume);

    for (int k = 1; k <= no_links; k++)
    {
        for(int m =1; m <= no_modes; m++)
        {
        Link[k].mode_MainVolume[m] = Link[k].mode_SubVolume[m];
        }
    }
    // FirstFWstatus(MainVolume, &fw_status);
    system_wide_travel_time = UpdateLinkCost(MainVolume);
    double gap = (system_wide_travel_time - system_least_travel_time) /
        (fmax(0.1, system_least_travel_time)) * 100;

    if (gap < 0)
    {
        int ii = 0;
    }

    printf("iter No = %d, sys. TT =  %lf, least TT =  %lf, gap = %f\n", iteration_no,
        system_wide_travel_time, system_least_travel_time, gap);




    for (iteration_no = 1; iteration_no < AssignIterations; iteration_no++)
    {
        system_least_travel_time = FindMinCostRoutes(MDMinPathPredLink);  // the one right before the assignment iteration 

        Assign(iteration_no, MDODflow, MDMinPathPredLink, SubVolume);
        VolumeDifference(SubVolume, MainVolume, SDVolume); /* Which yields the search direction. */
        Lambda = LinksSDLineSearch(MainVolume, SDVolume);

        // MSA options
     //   Lambda = 1.0 / (iteration_no + 1);
        // UpdateFWstatus(MainVolume, SDVolume, &fw_status);



 

        UpdateVolume(MainVolume, SDVolume, Lambda);
        system_wide_travel_time = UpdateLinkCost(MainVolume);

        //system_least_travel_time = FindMinCostRoutes(MinPathPredLink);  // the one right after the updated link cost 

        gap = (system_wide_travel_time - system_least_travel_time) /
            (fmax(0.1, system_least_travel_time)) * 100;


        if (gap < 0)
        {
            int ii = 0;
        }

        printf("iter No = %d, Lambda = %f, sys. TT =  %lf, least TT =  %lf, gap = %f\n",
            iteration_no, Lambda, system_wide_travel_time, system_least_travel_time, gap);

        //for (int k = 1; k <= no_links; k++)
        //{
        //    fprintf(logfile, "%d,%d,%d,%d,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf\n",
        //        iteration_no, k, Link[k].external_from_node_id, Link[k].external_to_node_id,
        //        MainVolume[k], Link[k].Ref_volume, Link[k].Capacity,
        //        MainVolume[k] / fmax(0.01, Link[k].Capacity), Link[k].FreeTravelTime,
        //        Link[k].Travel_time, Link[k].Travel_time - Link[k].FreeTravelTime);
        //}


    }

    //for (int k = 1; k <= no_links; k++)
    //{
    //    fprintf(logfile, "%d,%d,%d,%d,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf\n", iteration_no, k,
    //        Link[k].external_from_node_id, Link[k].external_to_node_id, MainVolume[k],
    //        Link[k].Capacity, MainVolume[k] / fmax(0.01, Link[k].Capacity),
    //        Link[k].FreeTravelTime, Link[k].Travel_time,
    //        Link[k].Travel_time - Link[k].FreeTravelTime);
    //}

    for (int k = 1; k <= no_links; k++)
    {
      
        double VMT, VHT, PMT, PHT;
        VMT = 0; VHT = 0;  PMT = 0; PHT = 0;
        for (int m = 1; m <= no_modes; m++)
        {
            VMT += MainVolume[k] * Link[k].length;
            VHT += MainVolume[k] * Link[k].Travel_time / 60.0;
            PMT += Link[k].mode_MainVolume[m]*g_mode_type_vector[m].occ * Link[k].length;
            PHT += Link[k].mode_MainVolume[m] * g_mode_type_vector[m].occ * Link[k].Travel_time / 60.0;

        }


        double P = 0;
        double vt2 = Link[k].Cutoff_Speed;
        double mu = Link[k].Lane_Capacity;
        double Severe_Congestion_P;
        double model_speed[300];
        double t0 = 0;
        double t2 = 0;
        double t3 = 0;
        double Q_gamma = 0;
        double congestion_ref_speed = 0; 
        double avg_queue_speed = 0;
        double avg_QVDF_period_speed = 0;
        double IncomingDemand = 0; 
        double DOC = 0;
        Link_QueueVDF(k, MainVolume[k], IncomingDemand, DOC, P,t0,t2,t3, vt2, mu, Q_gamma, congestion_ref_speed, avg_queue_speed, avg_QVDF_period_speed, Severe_Congestion_P, model_speed);

        fprintf(link_performance_file, "%d,%d,%d,%d,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,",
            iteration_no, Link[k].link_id, Link[k].external_from_node_id, Link[k].external_to_node_id,
            MainVolume[k], Link[k].Ref_volume, Link[k].Link_Capacity, IncomingDemand, DOC, Link[k].FreeTravelTime,
            Link[k].Travel_time, Link[k].VDF_Alpha, Link[k].VDF_Beta, Link[k].VDF_plf, Link[k].length / fmax(Link[k].Travel_time / 60.0, 0.001), Link[k].Travel_time - Link[k].FreeTravelTime);

        fprintf(link_performance_file, "%2lf,%2lf,%2lf,%2lf,", VMT, VHT, PMT, PHT);

        fprintf(link_performance_file, "\"%s\",",
            Link[k].geometry.c_str());

        for (int m = 1; m <= no_modes; m++)
            fprintf(link_performance_file, "%2lf,", Link[k].mode_MainVolume[m]);

        fprintf(link_performance_file, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,", P, t0, t2, t3, vt2, mu, Q_gamma, Link[k].free_speed, Link[k].Cutoff_Speed,
            congestion_ref_speed, avg_queue_speed, avg_QVDF_period_speed, Severe_Congestion_P);
        for (int t = demand_period_starting_hours * 60; t < demand_period_ending_hours * 60; t += 5)
        {
            int t_interval = t / 5;
            double speed = model_speed[t_interval];
            fprintf(link_performance_file, "%.3f,", speed);
        }



       fprintf(link_performance_file, "\n"); 
    }

    free(MainVolume);
    free(SubVolume);
    Free_3D((void***)MDMinPathPredLink, no_modes, no_zones, no_nodes);

    Close();

    fclose(link_performance_file);
    fclose(logfile);  // Close the log file when you're done with it.

    return 0;
}

static void Init(int int_no_modes, int input_no_zones)
{
    // tuiInit(tuiFileName);
    InitLinks();
    InitODflow(int_no_modes, input_no_zones);
    InitLineSearch();
}

static void Close()
{
    StatusMessage("General", "Closing all modules");
    // tuiClose(tuiFileName);
    CloseLinks();
    CloseODflow();
    CloseLineSearch();
}

static void InitODflow(int int_no_modes, int input_no_zones)
{

    double Factor = 1.0;

    // tuiGetInputFileName("OD flow file name", TRUE, ODflowFileName);
    StatusMessage("General", "Reading OD flow file");
    Read_ODflow(&TotalODflow, &int_no_modes, &input_no_zones);
}

static void CloseODflow(void)
{


    Free_3D((void***)MDODflow, no_modes, no_zones, no_zones);
    Free_3D((void***)MDRouteCost, no_modes, no_zones, no_zones);

}

void* Alloc_1D(int dim1, size_t size)
{
    void* Array;

    Array = (void*)calloc(dim1 + 1, size);
    if (Array == NULL)
    {
        ExitMessage("Can not allocate memory for single dimension array of size %d. \n", dim1);
    }
    return Array;
}

void** Alloc_2D(int dim1, int dim2, size_t size)
{
    void** Array;
    int i;

    Array = (void**)calloc(dim1 + 1, sizeof(void*));
    if (Array == NULL)
    {
        ExitMessage("Can not allocate memory for two-dimensions array of size %d by %d. \n", dim1,
            dim2);
    }
    for (i = 1; i <= dim1; i++)
    {
        Array[i] = (void*)calloc(dim2 + 1, size);
        if (Array[i] == NULL)
        {
            ExitMessage("Can not allocate memory for two-dimensions array of size %d by %d. \n",
                dim1, dim2);
        }
    }
    return (Array);
}

void Free_2D(void** Array, int dim1, int dim2)
{
    int i;
    void* p;

    for (i = 1; i <= dim1; i++)
    {
        p = Array[i];
        free(p);
    }
    free(Array);
}

void*** Alloc_3D(int dim1, int dim2, int dim3, size_t size)
{
    void*** Array; // This is the pointer to the 3D array
    int i, j;

    // Allocate memory for the 1D array of pointers to 2D arrays
    Array = (void***)calloc(dim1 + 1, sizeof(void**));
    if (Array == NULL)
    {
        ExitMessage("Can not allocate memory for three-dimensional array of size %d by %d by %d. \n", dim1, dim2, dim3);
    }

    // Loop through each of the 2D arrays
    for (i = 1; i <= dim1; i++)
    {
        // Allocate memory for each 2D array (which is a 1D array of pointers to 1D arrays)
        Array[i] = (void**)calloc(dim2 + 1, sizeof(void*));
        if (Array[i] == NULL)
        {
            ExitMessage("Can not allocate memory for two-dimensional array of size %d by %d. \n", dim1, dim2);
        }

        // Loop through each of the 1D arrays inside the 2D arrays
        for (j = 1; j <= dim2; j++)
        {
            // Allocate memory for each 1D array of size dim3
            Array[i][j] = (void*)calloc(dim3 + 1, size);
            if (Array[i][j] == NULL)
            {
                ExitMessage("Can not allocate memory for one-dimensional array of size %d. \n", dim3);
            }
        }
    }

    return Array;
}


void Free_3D(void*** Array, int dim1, int dim2, int dim3)
{
    int i, j;
    void* p;

    // Free the innermost arrays (1D arrays)
    for (i = 1; i <= dim1; i++)
    {
        for (j = 1; j < dim2; j++)
        {
            p = Array[i][j];
            free(p);
        }
        // Free the 2D array (array of pointers to 1D arrays)
        free(Array[i]);
    }

    // Free the outermost array (array of pointers to 2D arrays)
    free(Array);
}

/* Internal functions */

double Link_Travel_Time(int k, double* Volume)
{
    double IncomingDemand = Volume[k]/fmax(0.01, Link[k].lanes) /fmax(0.001, demand_period_ending_hours - demand_period_starting_hours)/ Link[k].VDF_plf;

    Link[k].Travel_time =
        Link[k].FreeTravelTime * (1.0 + Link[k].VDF_Alpha * (pow(IncomingDemand / fmax(0.1,Link[k].Link_Capacity), Link[k].VDF_Beta)));
    return (Link[k].Travel_time);
}


double Link_QueueVDF(int k, double Volume, double &IncomingDemand, double &DOC, double &P, double &t0, double &t2, double &t3, double& vt2, double& Q_mu, double &Q_gamma, double& congestion_ref_speed,
    double& avg_queue_speed, double &avg_QVDF_period_speed, double& Severe_Congestion_P, double model_speed[300])
{

    IncomingDemand = Volume / fmax(0.01, Link[k].lanes)/fmax(0.001, demand_period_ending_hours - demand_period_starting_hours) / Link[k].VDF_plf;
    DOC = IncomingDemand / fmax(0.1, Link[k].Lane_Capacity);

    double Travel_time =
        Link[k].FreeTravelTime * (1.0 + Link[k].VDF_Alpha * (pow(DOC, Link[k].VDF_Beta)));

    congestion_ref_speed = Link[k].Cutoff_Speed;
    if (DOC < 1)
        congestion_ref_speed = (1 - DOC) * Link[k].free_speed + DOC * Link[k].Cutoff_Speed;


    //step 3.2 calculate speed from VDF based on D/C ratio
    avg_queue_speed = congestion_ref_speed / (1.0 + Link[k].VDF_Alpha * pow(DOC, Link[k].VDF_Beta));
 

    P = Link[k].Q_cd * pow(DOC, Link[k].Q_n);  // applifed for both uncongested and congested conditions

    double H = demand_period_ending_hours - demand_period_starting_hours;
  
    if (P > H)
        avg_QVDF_period_speed = avg_queue_speed;
    else
        avg_QVDF_period_speed = P / H * avg_queue_speed + (1.0 - P / H) * (congestion_ref_speed + Link[k].free_speed) / 2.0;


    double base = Link[k].Q_cp * pow(P, Link[k].Q_s) + 1.0;
    vt2 = Link[k].Cutoff_Speed / fmax(0.001, base);

    t2 = (demand_period_starting_hours + demand_period_ending_hours) / 2.0; 
    t0 = t2 - 0.5 * P;
    t3 = t2 + 0.5 * P;

    Q_mu = std::min(Link[k].Lane_Capacity, IncomingDemand / std::max(0.01, P));

    //use  as the lower speed compared to 8/15 values for the congested states
    double RTT = Link[k].length / fmax(0.01, congestion_ref_speed);
    double wt2 = Link[k].length / vt2 - RTT; // in hour


    //step 5 compute gamma parameter is controlled by the maximum queue
    Q_gamma = wt2 * 64 * Q_mu / pow(P, 4);  // because q_tw = w*mu =1/4 * gamma (P/2)^4, => 1/vt2 * mu = 1/4 * gamma  * (P/2)^4


    double td_w = 0;
    //step scan the entire analysis period
    Severe_Congestion_P = 0;


    for (int t_in_min = demand_period_starting_hours * 60; t_in_min <= demand_period_ending_hours * 60; t_in_min += 5)  // 5 min interval
    {
        int t_interval = t_in_min / 5;
        double t = t_in_min / 60.0;  // t in hour
        double td_queue = 0;
        double td_speed = 0;
        model_speed[t_interval] = Link[k].free_speed;

        if (t0 <= t && t <= t3)  // within congestion duration P
        {
            //1/4*gamma*(t-t0)^2(t-t3)^2
            td_queue = 0.25 * Q_gamma * pow((t - t0), 2) * pow((t - t3), 2);
            td_w = td_queue / fmax(0.001, Q_mu);
            //L/[(w(t)+RTT_in_hour]
            td_speed = Link[k].length / (td_w + RTT);
        }
        else if (t < t0) //outside
        {
            td_queue = 0;
            double factor = (t - demand_period_starting_hours) / fmax(0.001, t0 - demand_period_starting_hours);
            td_speed = (1 - factor) * Link[k].free_speed + factor * fmax(congestion_ref_speed, avg_queue_speed);
        }
        else  // t> t3
        {
            td_queue = 0;
            double factor = (t - t3) / fmax(0.001, demand_period_ending_hours - t3);
            td_speed = (1 - factor) * fmax(congestion_ref_speed, avg_queue_speed) + (factor)*Link[k].free_speed;
        }

        // dtalog.output() << "td_queue t" << t << " =  " << td_queue << ", speed =" << td_speed << '\n';
        // g_DTA_log_file << "td_queue t" << t << " =  " << td_queue << ", speed =" << td_speed << '\n';



        if (t_in_min <= 410)
        {
            int idebug = 1;
        }
        double td_flow = 0; // default: get_volume_from_speed(td_speed, vf, k_critical, s3_m);
        model_speed[t_interval] = td_speed;

        if (td_speed < Link[k].free_speed * 0.5)
            Severe_Congestion_P += 5.0 / 60;  // 5 min interval
    }

    return P;
}

double Link_Travel_Time_Integral(int k, double* Volume)
{
    double IncomingDemand = Volume[k] / fmax(0.001, demand_period_ending_hours - demand_period_starting_hours) / Link[k].VDF_plf;

    if (Link[k].VDF_Beta >= 0.0)
               return (Volume[k] * Link[k].FreeTravelTime *
                (1.0 + (Link[k].BoverC / (Link[k].VDF_Beta + 1)) * pow(IncomingDemand, Link[k].VDF_Beta + 1)));
    else
        return 0.0;
}

double Link_Travel_Time_Der(int k, double* Volume)
{
    if (Link[k].VDF_Beta == 0.0)
        return 0.0;
    else
        return (Link[k].FreeTravelTime * Link[k].BoverC * Link[k].VDF_Beta *
            pow(Volume[k], (Link[k].VDF_Beta - 1)));
}

double AdditionalCost(int k, int m)
{
    double AddCost = 0;

    AddCost = Link[k].mode_Toll[m] /g_mode_type_vector[m].vot * 60.0;

    return AddCost;
}

double Link_GenCost(int k, double* Volume)
{
    	return (Link[k].mode_AdditionalCost[1] + Link_Travel_Time(k, Volume));
}

double LinkCost_Integral(int k, double* Volume)
{
    return (Link[k].mode_AdditionalCost[1] * Volume[k] + Link_Travel_Time_Integral(k, Volume));
}

double Link_GenCostDer(int k, double* Volume)
{
    return (Link_Travel_Time_Der(k, Volume));
}

/* External functions */

void ClearVolume(double* VolumeArray)
{
    int k;
    for (k = 1; k <= no_links; k++)
        VolumeArray[k] = 0.0;
}

void VolumeDifference(double* Volume1, double* Volume2, double* Difference)
{
    int k;
    for (k = 1; k <= no_links; k++)
        Difference[k] = Volume1[k] - Volume2[k];
}

void UpdateVolume(double* MainVolume, double* SDVolume, double Lambda)
{
    int k;
    for (k = 1; k <= no_links; k++)
    {
        MainVolume[k] += Lambda * SDVolume[k];
    }


    for (int k = 1; k <= no_links; k++)
    {
        for (int m = 1; m <= no_modes; m++)
        {
            Link[k].mode_MainVolume[m] = (1- Lambda)* Link[k].mode_MainVolume[m]  + Lambda * Link[k].mode_SubVolume[m];
        }
    }
}

void UpdateLinkAdditionalCost(void)
{
    int k;

    for (k = 1; k <= no_links; k++)
        for (int m = 1; m <= no_modes; m++)
            AdditionalCost(k,m);
}

double UpdateLinkCost(double* MainVolume)
{
    int k;
    double system_wide_travel_time = 0;

    for (k = 1; k <= no_links; k++)
    {
        Link[k].Travel_time = Link_Travel_Time(k, MainVolume);

        Link[k].GenCost = Link_GenCost(k, MainVolume);
        system_wide_travel_time += (MainVolume[k] * Link[k].Travel_time);
    }

    return system_wide_travel_time;
}

void UpdateLinkCostDer(double* MainVolume)
{
    int k;

    for (k = 1; k <= no_links; k++)
    {
        Link[k].GenCostDer = Link_GenCostDer(k, MainVolume);
    }
}

void GetLinkTravelTimes(double* Volume, double* TravelTime)
{
    int k;

    for (k = 1; k <= no_links; k++)
    {
        TravelTime[k] = Link_Travel_Time(k, Volume);
    }
}

double TotalLinkCost(double* Volume)
{
    int k;
    double Sum = 0;

    for (k = 1; k <= no_links; k++)
        Sum += Link[k].GenCost * Volume[k];
    return Sum;
}

double OFscale = 1.0;

double OF_Links(double* MainVolume)
{
    int k;
    double Sum = 0;

    for (k = 1; k <= no_links; k++)
        Sum += LinkCost_Integral(k, MainVolume);

    return Sum / OFscale;
}

double OF_LinksDirectionalDerivative(double* MainVolume, double* SDVolume, double Lambda)
{
    int k;
    double* Volume;
    double LinkCostSum = 0;

    Volume = (double*)Alloc_1D(no_links, sizeof(double));

    for (k = 1; k <= no_links; k++)
    {
        Volume[k] = MainVolume[k] + Lambda * SDVolume[k];
    }
    for (k = 1; k <= no_links; k++)
    {
        LinkCostSum += Link_GenCost(k, Volume) * SDVolume[k];
    }

    free(Volume);
    return LinkCostSum / OFscale;
}

/* Initialization functions */
struct CLink {
    int link_id;
    int internal_from_node_id;
    int internal_to_node_id;
    int length;
    int lanes;
    double capacity;
    int free_speed;
};





void ReadLinks()
{
    CDTACSVParser parser_link;
    std::vector<CLink> links;

    std::string line;

    if (parser_link.OpenCSVFile("link.csv", true))
    {
        int line_no = 0;

        int k = 1;  // link start from 1

        while (parser_link.ReadRecord())  // if this line contains [] mark, then we will also read
            // field headers.
        {
            std::string value;
            // CLink link;
            int lanes = 0;
            float capacity = 0;
            float free_speed = 10;
            // Read link_id
            parser_link.GetValueByFieldName("from_node_id", Link[k].external_from_node_id);
            parser_link.GetValueByFieldName("to_node_id", Link[k].external_to_node_id);
            parser_link.GetValueByFieldName("link_id", Link[k].link_id);

            

            Link[k].internal_from_node_id = Link[k].external_from_node_id;
            Link[k].internal_to_node_id = Link[k].external_to_node_id;

            if (g_map_external_node_id_2_node_seq_no.find(Link[k].external_from_node_id) !=
                g_map_external_node_id_2_node_seq_no.end())
                Link[k].internal_from_node_id =
                g_map_external_node_id_2_node_seq_no[Link[k].external_from_node_id];

            if (g_map_external_node_id_2_node_seq_no.find(Link[k].external_to_node_id) !=
                g_map_external_node_id_2_node_seq_no.end())
                Link[k].internal_to_node_id =
                g_map_external_node_id_2_node_seq_no[Link[k].external_to_node_id];

            parser_link.GetValueByFieldName("length", Link[k].length);
            parser_link.GetValueByFieldName("ref_volume", Link[k].Ref_volume);

            parser_link.GetValueByFieldName("lanes", lanes);
            parser_link.GetValueByFieldName("capacity", capacity);

            Link[k].lanes = lanes;
            Link[k].Lane_Capacity = capacity;
            Link[k].Link_Capacity = lanes * capacity;

            parser_link.GetValueByFieldName("free_speed", free_speed);
            parser_link.GetValueByFieldName("allowed_use", Link[k].allowed_uses);



            for (int m = 1; m <= no_modes; m++)
            {
                Link[k].mode_allowed_use[m] = 1;
                Link[k].mode_MainVolume[m] = 0;
                Link[k].mode_SubVolume[m] = 0;
                Link[k].mode_Toll[m] = 0;          
                Link[k].mode_AdditionalCost[m] = 0;

            }

            if (Link[k].allowed_uses.size() > 0 && Link[k].allowed_uses != "all")
            {
                for (int m =1; m<= no_modes; m++)
                {
                    if (Link[k].allowed_uses.find(g_mode_type_vector[m].mode_type) != std::string::npos)  // otherwise, only an agent type is listed in this "allowed_uses", then this agent type is allowed to travel on this link
                    {
                        Link[k].mode_allowed_use[m] = 1;  // found 
                    }
                    else
                    {
                        Link[k].mode_allowed_use[m] = 0; 
                    }
                }


            }



            // Read internal_from_node_id

            // Read length

            // Read capacity

            Link[k].FreeTravelTime = Link[k].length / free_speed * 60.0;

            parser_link.GetValueByFieldName("VDF_alpha", Link[k].VDF_Alpha);
            parser_link.GetValueByFieldName("VDF_beta", Link[k].VDF_Beta);
            parser_link.GetValueByFieldName("VDF_plf", Link[k].VDF_plf);

    
            for (int m = 1; m <= no_modes; m++)
            {
                char CSV_field_name[50];
                sprintf(CSV_field_name, "toll_%s", g_mode_type_vector[m].mode_type.c_str());
                parser_link.GetValueByFieldName(CSV_field_name, Link[k].mode_Toll[m], false, false);

                Link[k].mode_AdditionalCost[m] = Link[k].mode_Toll[m] / g_mode_type_vector[m].vot * 60.0;
            }



            if (capacity > 0)
                Link[k].BoverC = Link[k].VDF_Alpha / pow(capacity* lanes, Link[k].VDF_Beta);
            else
                Link[k].BoverC = 0;


            parser_link.GetValueByFieldName("VDF_cp", Link[k].Q_cp);
            parser_link.GetValueByFieldName("VDF_cd", Link[k].Q_cd);
            parser_link.GetValueByFieldName("VDF_n", Link[k].Q_n);
            parser_link.GetValueByFieldName("VDF_s", Link[k].Q_s);
   
            Link[k].free_speed = free_speed;
            Link[k].Cutoff_Speed = free_speed * 0.75;  // use 0.75 as default ratio, when free_speed = 70 mph, Cutoff_Speed = 52.8 mph in I-10 data set
            parser_link.GetValueByFieldName("geometry", Link[k].geometry, false);
            k++;
        }

        parser_link.CloseCSVFile();
    }

    // int count, k, internal_from_node_id, internal_to_node_id, Type;
    // double Capacity, length, FreeTravelTime, B, VDF_Beta, Speed, Toll;
    // const char* LinkArgumentDescription[10] = {
    // "internal_from_node_id","head","capacity","length", 	"free flow travel time","VDF_Beta","speed
    //limit","toll","link type","semicolomn - for end of link" };
    ///* Descriptions of the various link record arguments, used for error messages. */

    // FILE* LinksFile2;
    // errno_t err = fopen_s(&LinksFile2, "link.csv", "r");
    // if (err) {
    //	printf("Failed to open file\n");
    //	return;
    // }

    // for (k = 1; ; k++)
    //{ // Loop until break
    //	count = fscanf_s(LinksFile2, "%d %d %lf %lf %lf %lf %lf %lf %lf %d %1[;]",
    //		&internal_from_node_id, &internal_to_node_id, &Capacity, &length, &FreeTravelTime,
    //		&B, &VDF_Beta, &Speed, &Toll, &Type, Semicolon, sizeof(Semicolon));
    //	if (count == -1)
    //	{
    //		// If we read to the end of the file, break out of the loop.
    //		if (feof(LinksFile2)) {
    //			break;
    //		}
    //		// If there was an error other than end-of-file, handle it here.
    //		else {
    //			printf("Read error on link %d\n", k);
    //			break;
    //		}
    //	}

    //	//if (VDF_Beta < 0)
    //	//{
    //	//std::cout <<	("Link file '%s', link %d VDF_Beta %lf < 0 is decreasing cost",
    //	//		LinksFileName, k, Link[k].VDF_Beta);
    //	//}

    //	if (Capacity > 0)
    //		Link[k].BoverC = B / pow(Capacity, VDF_Beta);
    //	else {
    //		Link[k].BoverC = 0;
    //		//InputWarning("link file '%s', link %d from %d to %d has %lf capacity \n",
    //		//	LinksFileName, k, internal_from_node_id, internal_to_node_id, Capacity);
    //	}

    //}

    // fclose(LinksFile2);
}

static void InitLinkPointers(char* LinksFileName)
{
    int k, Node, internal_from_node_id;

    FirstLinkFrom = (int*)Alloc_1D(no_nodes, sizeof(int));
    LastLinkFrom = (int*)Alloc_1D(no_nodes, sizeof(int));

    FirstLinkFrom[1] = 1;
    Node = 1;

    for (k = 1; k <= no_links; k++)
    {
        internal_from_node_id = Link[k].internal_from_node_id;
        if (internal_from_node_id == Node)
            continue;
        else if (internal_from_node_id == Node + 1)
        {
            LastLinkFrom[Node] = k - 1;
            Node = internal_from_node_id;
            FirstLinkFrom[Node] = k;
        }

        /**********************
        CHECKING FOR SORT ERRORS AND GAPS IN THE LINKS FILE.
        *********************/
        else if (internal_from_node_id < Node)
        {
            // ExitMessage("Sort error in link file '%s': a link from node %d was found after "
            //	"a link from node %d \n", LinksFileName, internal_from_node_id, Node);
        }
        else if (internal_from_node_id > Node + 1)
        {
            // InputWarning("link file '%s' has no links out from "
            //	"nodes %d through %d. \n", LinksFileName, Node + 1, internal_from_node_id - 1);
            LastLinkFrom[Node] = k - 1;
            for (Node++; Node < internal_from_node_id; Node++)
            {
                FirstLinkFrom[Node] = 0;
                LastLinkFrom[Node] = -1;
            }
            FirstLinkFrom[Node] = k; /* Node equals internal_from_node_id now. */
        }
    }

    if (Node == no_nodes)
    {
        LastLinkFrom[Node] = no_links; /* Now Node equals no_nodes in any case */
    }
    else
    {
        // InputWarning("link file '%s' has no links out from "
        //	"nodes %d through %d. \n", LinksFileName, Node + 1, no_nodes);
        LastLinkFrom[Node] = k - 1;
        for (Node++; Node <= no_nodes; Node++)
        {
            FirstLinkFrom[Node] = 0;
            LastLinkFrom[Node] = -1;
        }
    }
}

void FindLinksTo(void)
{
    int Node, k;

    LinksTo = (sorted_list*)Alloc_1D(no_nodes, sizeof(sorted_list*));
    for (Node = 1; Node <= no_nodes; Node++)
        LinksTo[Node] = NULL;

    for (k = 1; k <= no_links; k++)
        ListAdd(k, &(LinksTo[Link[k].internal_to_node_id]));
}
void InitLinks()
{
    char LinksFileName[100] = "link.csv";
    char* InterFlowFileName;
    FILE* LinksFile;
    int k;

    Link = (struct link_record*)Alloc_1D(no_links, sizeof(struct link_record));
    ReadLinks();
    FindLinksTo();
    InitLinkPointers(LinksFileName);
    UpdateLinkAdditionalCost();
}

void CloseLinks(void)
{
    int Node;

    free(Link);
    free(FirstLinkFrom);
    free(LastLinkFrom);
    for (Node = 1; Node <= no_nodes; Node++)
        ListFree(LinksTo[Node]);
    free(LinksTo);
}

/* Report functions */

#define MAX_NO_BISECTITERATION 1000 /* Avoids inifinite loops in the second part */

static int MinLineSearchIterations = 5;
static int ActualIterations = 0;
static double LastLambda = 1.0;

void InitLineSearch(void)
{
    // tuiGetInt("Line search iterations {LSITE}", FALSE, &MinLineSearchIterations);
}

void CloseLineSearch(void) {}

double LinksSDLineSearch(double* MainVolume, double* SDVolume)
{
    int n;
    double lambdaleft, lambdaright, lambda;
    double grad;

    grad = OF_LinksDirectionalDerivative(MainVolume, SDVolume, 0.0);
    StatusMessage("Line search step", "0.0");
    StatusMessage("Line search grad", "%lf", grad);
    if (grad >= 0)
    {
        LastLambda = 0.0;
        return (0.0);
    }

    grad = OF_LinksDirectionalDerivative(MainVolume, SDVolume, 1.0);
    StatusMessage("Line search step", "1.0");
    StatusMessage("Line search grad", "%lf", grad);
    if (grad <= 0)
    {
        LastLambda = 1.0;
        return (1.0);
    }

    lambdaleft = 0;
    lambdaright = 1;
    lambda = 0.5;

    for (n = 1; n <= MinLineSearchIterations; n++)
    {
        grad = OF_LinksDirectionalDerivative(MainVolume, SDVolume, lambda);

        if (grad <= 0.0)
            lambdaleft = lambda;
        else
            lambdaright = lambda;

        lambda = 0.5 * (lambdaleft + lambdaright);
    }
    for (; ((lambdaleft == 0) && (n <= MAX_NO_BISECTITERATION)); n++)
    {
        grad = OF_LinksDirectionalDerivative(MainVolume, SDVolume, lambda);


        if (grad <= 0.0)
            lambdaleft = lambda;
        else
            lambdaright = lambda;

        lambda = 0.5 * (lambdaleft + lambdaright);
    }
    ActualIterations = n - 1;
    LastLambda = lambdaleft;
    return lambdaleft;
}

void print_ls_header(FILE* fp)
{
    fprintf(fp, "LS iterations \tStep size \t");
}

void ls_report(FILE* fp)
{
    fprintf(fp, "%d \t%g \t", ActualIterations, LastLambda);
}

#define BUFFERSIZE 100

static struct list_item* NewListItem(int value)
{
    struct list_item* new_item;

    new_item = (struct list_item*)malloc(sizeof(struct list_item));
    if (new_item == NULL)
    {
        printf("Error in memory allocation of list item. \n");
        exit(EXIT_FAILURE);
    }
    (*new_item).value = value;
    (*new_item).next_item = NULL;

    return (new_item);
}

/* Check if value is in list */
int ListFind(int Value, sorted_list list)
{
    struct list_item* item;

    for (item = list; item != NULL && (*item).value < Value; item = (*item).next_item)
        ;
    if (item != NULL && (*item).value == Value)
        return 1;
    else
        return 0;
}

/* Try to add a new item to the list.
It is assumed that value is greater then the first value in the list.
Return the list item that contains "value".*/

static struct list_item* InListAdd(int value, struct list_item* item)
{
    struct list_item* new_item, * remaining_list;

    /* If there are no more items in the list, add the new value in the end. */
    if ((*item).next_item == NULL)
    {
        new_item = NewListItem(value);
        (*item).next_item = new_item;
        return (new_item);
    }

    remaining_list = (*item).next_item;

    if (value == (*remaining_list).value)
    {
        return (remaining_list);
    }

    else if (value < (*remaining_list).value)
    {
        new_item = NewListItem(value);
        (*new_item).next_item = remaining_list;
        (*item).next_item = new_item;
        return (new_item);
    }

    else
    {
        return (InListAdd(value, remaining_list));
    }
}

void ListAdd(int value, sorted_list* list)
{
    struct list_item* new_item;

    if (*list == NULL)
    {
        *list = NewListItem(value);
    }
    /* Adding the value in the beginning is a special case, all others are handled by recursion. */
    else if (value < (**list).value)
    {
        new_item = NewListItem(value);
        (*new_item).next_item = (sorted_list)*list;
        *list = new_item;
    }
    else if (value > (**list).value)
    {
        InListAdd(value, *list);
    }

    /* If the new value is equal to the first value then there is nothing to do. */
}

/* Try to remove an item from a list.
It is assumed that value is greater then the first value in the list.
If value is found in the list, return 1 and remove it;
otherwise return 0 and do not change the list. */

static int InListRemove(int value, struct list_item* item)
{
    struct list_item* remaining_list;

    if ((*item).next_item == NULL)
        return (0);

    remaining_list = (*item).next_item;
    if (value == (*remaining_list).value)
    {
        (*item).next_item = (*remaining_list).next_item;
        free(remaining_list);
        return (1);
    }
    else if (value < (*remaining_list).value)
    {
        return (0);
    }
    else
    {
        return (InListRemove(value, remaining_list));
    }
}

int ListRemove(int value, sorted_list* list)
{
    struct list_item* item_to_free;

    if (*list == NULL)
    {
        return (0);
    }
    else if (value < (**list).value)
    {
        return (0);
    }
    else if (value == (**list).value)
    {
        item_to_free = (sorted_list)*list;
        *list = (**list).next_item;
        free(item_to_free);
        return (1);
    }
    else
    { /* value > (**list).value */
        return (InListRemove(value, *list));
    }
}

static void InListFree(struct list_item* item)
{
    if ((*item).next_item != NULL)
        InListFree((*item).next_item);
    free(item);
}

void ListFree(sorted_list list)
{
    if (list != NULL)
        InListFree(list);
}

/* Creat a copy of a list. */
sorted_list ListCopy(sorted_list list)
{
    sorted_list new_list;
    struct list_item* item, * new_item;

    /* If the first list is empty, return an empty list. */
    if (list == NULL)
        return (NULL);
    else
    {
        item = list;
        new_list = new_item = NewListItem((*item).value);
        item = (*item).next_item;
        while (item != NULL)
        {
            (*new_item).next_item = NewListItem((*item).value);
            new_item = (*new_item).next_item;
            item = (*item).next_item;
        }
        return (new_list);
    }
}

/* Merge list1 into list2. */
void ListMerge(sorted_list list1, sorted_list* list2)
{
    struct list_item* item1, * item2;

    /* If the first list is empty, there is nothing to do. */
    if (list1 == NULL)
        return;

    /* If the second list empty, set it to a copy of list1. */
    if (*list2 == NULL)
    {
        *list2 = ListCopy(list1);
    }

    else
    {
        /* InListAdd assumes that the first value in the list is lower then the new value. */
        /* Therefore, if the first value in list1 is smaller, make it the first value in list2.
         * Other values wil be larger. */
        if ((*list1).value < (**list2).value)
        {
            item2 = NewListItem((*list1).value);
            (*item2).next_item = (sorted_list)*list2;
            *list2 = item2;
            item1 = (*list1).next_item;
        }
        /* if the first value in list1 and list2 are equal, skip the first value in list1. Other
         * values wil be larger. */
        else if ((*list1).value == (**list2).value)
        {
            item2 = (sorted_list)*list2;
            item1 = (*list1).next_item;
        }
        /* Otherwise, if the first value in list1 is greater, start from the first value of list1.
         */
        else
        {
            item2 = (sorted_list)*list2;
            item1 = list1;
        }

        /* Add the remaining items of list1 one by one, continue always from the last item added.
        (previous items have lower values then the remaining values of list1.) */
        for (; item1 != NULL; item1 = (*item1).next_item)
            item2 = InListAdd((*item1).value, item2);
    }
}

sorted_list ListIntersect(sorted_list list1, sorted_list list2)
{
    struct list_item* item1, * item2;
    sorted_list new_list = NULL;

    item1 = list1;
    item2 = list2;
    while (item1 != NULL && item2 != NULL)
    {
        if ((*item1).value < (*item2).value)
            item1 = (*item1).next_item;
        else if ((*item2).value < (*item1).value)
            item2 = (*item2).next_item;
        else
        { /* (*item1).value==(*item2).value */
            ListAdd((*item1).value, &new_list);
            item1 = (*item1).next_item;
            item2 = (*item2).next_item;
        }
    }
    return (new_list);
}

/* Find all elements in list1 that do not show up in list2. */
sorted_list ListDifference(sorted_list list1, sorted_list list2)
{
    struct list_item* item1, * item2;
    sorted_list new_list = NULL;

    item1 = list1;
    item2 = list2;
    while (item1 != NULL && item2 != NULL)
    {
        if ((*item1).value < (*item2).value)
        {
            ListAdd((*item1).value, &new_list);
            item1 = (*item1).next_item;
        }
        else if ((*item2).value < (*item1).value)
        {
            item2 = (*item2).next_item;
        }
        else
        { /* (*item1).value = (*item2).value */
            item1 = (*item1).next_item;
            item2 = (*item2).next_item;
        }
    }

    while (item1 != NULL)
    { /* Add remaining items of list1, if there are any. */
        ListAdd((*item1).value, &new_list);
        item1 = (*item1).next_item;
    }

    return (new_list);
}

int ListsAreEqual(sorted_list list1, sorted_list list2)
{
    struct list_item* item1, * item2;

    for (item1 = list1, item2 = list2; item1 != NULL; item1 = (*item1).next_item)
    {
        if (item2 == NULL)
            return (0);
        else if ((*item1).value != (*item2).value)
            return (0);
        item2 = (*item2).next_item;
    }

    if (item2 != NULL)
        return (0);
    else
        return (1);
}

int ListSize(sorted_list list)
{
    struct list_item* item;
    int i;

    for (i = 0, item = list; item != NULL; i++, item = (*item).next_item)
        ;

    return (i);
}

void StatusMessage(const char* group, const char* format, ...)
{
    double new_time;
}

void Read_ODtable(double*** ODtable, int no_zones)
{
    char ch, Coloumn[2], Semicoloumn[2]; /* Reserve room for the '\0' in the fscanf_s. */
    int Orig, Dest, NewOrig, NewDest;
    double Value;


    for(int m = 1; m <= no_modes; m++)
    { 
    FILE* file;
    fopen_s(&file, g_mode_type_vector[m].demand_file.c_str(), "r");
    printf("read demand file %s\n", g_mode_type_vector[m].demand_file.c_str());

    if (file == NULL)
    {
        printf("Failed to open demand file %s\n", g_mode_type_vector[m].demand_file.c_str());
        return;
    }

    int o_zone_id, d_zone_id;
    double volume;

    // Skip the header line
    char header[100];
    if (fgets(header, sizeof(header), file) == NULL)
    {
        printf("Failed to read header\n");
        return;
    }

    int line_count = 0;
    double total_volume = 0;
    // Read the data
    int result;
    while ((result = fscanf(file, "%d,%d,%lf", &o_zone_id, &d_zone_id, &volume)) != EOF)
    {
        if (result == 3)  // we have read all the 3 values correctly
        {
            if (line_count <= 3)
            {
                printf("o_zone_id: %d, d_zone_id: %d, volume: %.4lf\n", o_zone_id, d_zone_id,
                    volume);

            }
            ODtable[m][o_zone_id][d_zone_id] = volume;
            total_volume += volume;
            line_count++;
        }
        else
        {
            printf("Error reading line %d\n", line_count);
            break;
        }
    }

    printf(" mode type = %s, total_volume = %f\n", g_mode_type_vector[m].mode_type.c_str(), total_volume);

    fclose(file);

    }
}

double Sum_ODtable(double*** ODtable, int no_zones)
{
    int Orig, Dest;
    double sum = 0.0;

    for (int m = 1; m <= no_modes; m++)
    for (Orig = 1; Orig <= no_zones; Orig++)
        for (Dest = 1; Dest <= no_zones; Dest++)
             sum += ODtable[m][Orig][Dest];

    return (sum);
}

void Read_ODflow(double* TotalODflow, int* no_modes, int* no_zones)
{
    FILE* ODflowFile;
    double RealTotal, InputTotal;

    MDODflow = (double***)Alloc_3D(*no_modes,*no_zones, *no_zones, sizeof(double));
    MDRouteCost = (double***)Alloc_3D(*no_modes, *no_zones, *no_zones, sizeof(double));

    Read_ODtable(MDODflow, *no_zones);

    RealTotal = (double)Sum_ODtable(MDODflow, *no_zones);

    *TotalODflow = (double)RealTotal;

}

void ExitMessage(const char* format, ...)
{
    va_list ap;

    // vprintf(format, ap);
    printf("\n");

    getchar();

    exit(EXIT_FAILURE);
}