// TAPLite.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <omp.h>
#define FLOAT_ACCURACY 1.0E-15

#define NO_COSTPARAMETERS 4
#define IVTT 0
#define OVTT 1
#define MONETARY 2
#define DIST 3

#define MIN(x,y)		( ((x)<(y)) ? (x) : (y) )
#define MAX(x,y)		( ((x)>(y)) ? (x) : (y) )
#define SQR(x) ((x)*(x))
#define FABS(x) ((x)>=0 ? (x) : (-x))

#define INVALID -1
#define VALID(x) ((x)!=-1)

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <map>
typedef double cost_vector[NO_COSTPARAMETERS];


struct link_record {
	int internal_from_node_id;
	int internal_to_node_id;

	int external_from_node_id;
	int external_to_node_id;

	double Capacity;
	double FreeTravelTime;
	double B;
	double BoverC;
	double Power;
	double Toll;
	double AdditionalCost;
	double Distance;
	double Speed;
//	int Type;
	double Delay;
	double Travel_time;  
	double GenCost;
	double GenCostDer;
	double Ref_volume; 
};


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

	CDTACSVParser() : Delimiter{ ',' }, IsFirstLineHeader{ true }, m_bSkipFirstLine{ false }, m_bDataHubSingleCSVFile{ false }, m_bLastSectionRead{ false }
	{
	}

	~CDTACSVParser()
	{
		if (inFile.is_open())
			inFile.close();
	}

	// inline member functions
	std::vector<std::string> GetHeaderVector()
	{
		return Headers;
	}
	void CloseCSVFile()
	{
		inFile.close();
	}

	void ConvertLineStringValueToIntegers();
	bool OpenCSVFile(std::string fileName, bool b_required);
	bool ReadRecord();
	bool ReadSectionHeader(std::string s);
	bool ReadRecord_Section();
	std::vector<std::string> ParseLine(std::string line);
	bool GetValueByFieldName(std::string field_name, std::string& value, bool required_field = true);
	template <class T> bool GetValueByFieldName(std::string field_name, T& value, bool required_field = true, bool NonnegativeFlag = true);
	template <class T> bool GetValueByKeyName(std::string field_name, T& value, bool required_field = true, bool NonnegativeFlag = true);

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

bool CDTACSVParser::OpenCSVFile(std::string  fileName, bool b_required)
{
	mFileName = fileName;
	inFile.open(fileName.c_str());

	if (inFile.is_open())
	{
		if (IsFirstLineHeader)
		{
			std::string  s;
			std::getline(inFile, s);
			std::vector<std::string > FieldNames = ParseLine(s);

			for (size_t i = 0; i < FieldNames.size(); i++)
			{
				std::string  tmp_str = FieldNames.at(i);
				size_t start = tmp_str.find_first_not_of(" ");

				std::string  name;
				if (start == std::string::npos)
				{
					name = "";
				}
				else
				{
					name = tmp_str.substr(start);
					//TRACE("%s,", name.c_str());
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
			//g_program_stop();
		}
		return false;
	}
}

bool CDTACSVParser::ReadRecord()
{
	LineFieldsValue.clear();

	if (inFile.is_open())
	{
		std::string  s;
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


std::vector<std::string > CDTACSVParser::ParseLine(std::string  line)
{
	std::vector<std::string > SeperatedStrings;
	std::string  subStr;

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

			if (n1 == std::string::npos && n2 == std::string::npos) //last field without double quotes
			{
				subStr = line;
				SeperatedStrings.push_back(subStr);
				break;
			}

			if (n1 == std::string::npos && n2 != std::string::npos) //last field with double quotes
			{
				size_t n3 = line.find_first_of('"', n2 + 1); // second double quote

				//extract content from double quotes
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
				else // comma is the last char in the line string, push an empty string to the back of vector
				{
					SeperatedStrings.push_back("");
					break;
				}
			}

			if (n1 != std::string::npos && n2 != std::string::npos && n2 < n1)
			{
				size_t n3 = line.find_first_of('"', n2 + 1); // second double quote
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

bool CDTACSVParser::GetValueByFieldName(std::string  field_name, std::string& value, bool required_field)
{
	if (FieldsIndices.find(field_name) == FieldsIndices.end())
	{
		if (required_field)
		{
			//dtalog.output() << "[ERROR] Field " << field_name << " in file " << mFileName << " does not exist. Please check the file." << '\n';
			//g_DTA_log_file << "[ERROR] Field " << field_name << " in file " << mFileName << " does not exist. Please check the file." << '\n';
			//g_program_stop();
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
		std::string  str_value = LineFieldsValue[index];

		if (str_value.length() <= 0)
		{
			return false;
		}

		value = str_value;
		return true;
	}
}
// Peiheng, 03/22/21, to avoid implicit instantiations in flash_dta.cpp and main_api.cpp for this template function only
// all the other non-inline functions are implemented in utils.cpp
template <class T>
bool CDTACSVParser::GetValueByFieldName(std::string field_name, T& value, bool required_field, bool NonnegativeFlag)
{
	if (FieldsIndices.find(field_name) == FieldsIndices.end())
	{
		if (required_field)
		{
			//dtalog.output() << "[ERROR] Field " << field_name << " in file " << mFileName.c_str() << " does not exist. Please check the file." << '\n';
			//g_DTA_log_file << "[ERROR] Field " << field_name << " in file " << mFileName.c_str() << " does not exist. Please check the file." << '\n';
			//g_program_stop();
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

		//if (required_field)
		//{
		//    if(NonnegativeFlag)
		//    {
		//        if (converted_value < 0)
		//            converted_value = 0;
		//    }
		//}

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


double** Read_ODflow(double* TotalODflow, int* no_zones);

int Minpath(int Orig, int* PredLink, double* Cost_to);
double FindMinCostRoutes(int** MinPathPredLink);
void Assign(double** ODflow, int** MinPathPredLink, double* Volume);

void InitLineSearch(void);
void CloseLineSearch(void);
double LinksSDLineSearch(double* MainVolume, double* SDVolume);

void print_ls_header(FILE* fp);
void ls_report(FILE* fp);


void StatusMessage(const char* group, const char* format, ...);
void ExitMessage(const char* format, ...);

int no_zones, no_nodes, no_links, FirstThruNode;
int	AssignIterations;
/* Gloabal variables */

double ** ODflow, TotalODflow;
double ** RouteCost;

/* Local Declarations */
/* void FW(void); Should there be a function for fw, or should it be included in main? */
static void Init(int input_no_zones);
static void Close();
static void InitODflow(int input_no_zones);
static void CloseODflow(void);
/* End of local declarations. */


FILE* logfile;
int shortest_path_log_flag = 0; 

FILE* link_performance_file;

#define INVALID -1			/* Represents an invalid value. */
#define WAS_IN_QUEUE -7			/* Shows that the node was in the queue before. (7 is for luck.) */


int Minpath(int Orig, int* PredLink, double* CostTo) {
	int node, now, NewNode, k, Return2Q_Count = 0;
	double NewCost;
	int* QueueNext;
	int QueueFirst, QueueLast;

	QueueNext = (int*)Alloc_1D(no_nodes, sizeof(int));

	for (node = 1; node <= no_nodes; node++) {
		QueueNext[node] = INVALID;
		CostTo[node] = 1.0e+15;
		PredLink[node] = INVALID;
	}

	now = Orig;
	QueueNext[now] = WAS_IN_QUEUE;
	PredLink[now] = INVALID;
	CostTo[now] = 0.0;

	QueueFirst = QueueLast = INVALID;


	while ((now != INVALID) && (now != WAS_IN_QUEUE)) {
		if (now >= FirstThruNode || now == Orig) {

			for (k = FirstLinkFrom[now]; k <= LastLinkFrom[now]; k++) {
				/* For every link that terminate at "now": */

				NewNode = Link[k].internal_to_node_id;
				NewCost = CostTo[now] + Link[k].GenCost;

				if (CostTo[NewNode] > NewCost) {
					/* If the new lable is better than the old one, correct it, and make sure that the new node to the queue. */

					CostTo[NewNode] = NewCost;
					PredLink[NewNode] = k;

					/* If the new node was in the queue before, add it as the first in the queue. */
					if (QueueNext[NewNode] == WAS_IN_QUEUE) {
						QueueNext[NewNode] = QueueFirst;
						QueueFirst = NewNode;
						if (QueueLast == INVALID)
							QueueLast = NewNode;
						Return2Q_Count++;
					}

					/* If the new node is not in the queue, and wasn't there before, add it at the end of the queue. */
					else if (QueueNext[NewNode] == INVALID && NewNode != QueueLast) {
						if (QueueLast != INVALID) { 					/*Usually*/
							QueueNext[QueueLast] = NewNode;
							QueueLast = NewNode;
						}
						else {			  /* If the queue is empty, initialize it. */
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
		if ((now == INVALID) || (now == WAS_IN_QUEUE))	break;

		QueueFirst = QueueNext[now];
		QueueNext[now] = WAS_IN_QUEUE;
		if (QueueLast == now) QueueLast = INVALID;
	}

	free(QueueNext);

	return(Return2Q_Count);
}



/* Find minimum cost routes .
Input: 	None
Output:	RouteCost - route generalized cost, by origin and destination
	MinPathSuccLink - trees of minimum cost routes, by destination and node. */

double  FindMinCostRoutes(int** MinPathPredLink) {
	int Orig, Dest;
	double* CostTo;
	double system_least_travel_time = 0;
	CostTo = (double*)Alloc_1D(no_nodes, sizeof(double));
	StatusMessage("Minpath", "Starting the minpath calculations.");

#pragma omp parallel for schedule(dynamic)
	for (Orig = 1; Orig <= no_zones; Orig++)
	{
	//	StatusMessage("Minpath", "Searching minpath for origin %6d.", Orig);
		Minpath(Orig, MinPathPredLink[Orig], CostTo);
		if (RouteCost != NULL) {
			for (Dest = 1; Dest <= no_zones; Dest++) {
				if (CostTo[Dest] < 0.0)
					ExitMessage("Negative cost %lg from Origin %d to Destination %d.",
						(double)CostTo[Dest], Orig, Dest);

				RouteCost[Orig][Dest] = CostTo[Dest];
#pragma omp critical
				{
				system_least_travel_time += RouteCost[Orig][Dest] * ODflow[Orig][Dest];
				}
			}
		}
	}
	free(CostTo);	StatusMessage("Minpath", "Found all minpath.");
	return system_least_travel_time;
}


/* Assign OD flows to links according to the routes in MinPathPredLink. */

void Assign(int Assignment_iteration_no, double** ODflow, int** MinPathPredLink, double* Volume)
{
	int Dest, Orig, k;
	int CurrentNode;
	double RouteFlow;



	//// Create a log file
	//FILE* logfile_od;
	//fopen_s(&logfile_od, "assignment_logfile_od.txt", "a+");
	//if (logfile_od == NULL) {
	//	printf("Error opening file!\n");
	//	return;
	//}

	for (k = 1; k <= no_links; k++)
		Volume[k] = 0.0;

	StatusMessage("Assign", "Starting assign.");


	for (Orig = 1; Orig <= no_zones; Orig++)
	{
	//	printf("Assign", "Assigning origin %6d.", Orig);
		for (Dest = 1; Dest <= no_zones; Dest++)
		{
			if (Dest == Orig) continue;

			RouteFlow = ODflow[Orig][Dest];
			if (RouteFlow == 0) continue;
			CurrentNode = Dest;
			double total_travel_time = 0;
			double total_distance = 0;
			double total_FFTT = 0;

			while (CurrentNode != Orig)
			{
				k = MinPathPredLink[Orig][CurrentNode];
				if (k == INVALID) {
					//Warning("A problem in mincostroutes.c (Assign): Invalid pred for node %d Orig %d \n\n", CurrentNode, Orig);
					break;
				}
				Volume[k] += RouteFlow;
				CurrentNode = Link[k].internal_from_node_id;

				total_travel_time += Link[k].Travel_time;
				total_distance += Link[k].Distance;
				total_FFTT += Link[k].FreeTravelTime;
				// Log the link and OD pair

				if(shortest_path_log_flag)
				{
				fprintf(logfile, "Iteration: %d, Origin: %d, Destination: %d, Route Flow: %f,Link: %d, from: %d, to: %d, tt: %f, FFTT: %f, delay: %f, distance: %f \n",
					Assignment_iteration_no, Orig, Dest, RouteFlow,
					k, Link[k].external_from_node_id, Link[k].external_to_node_id, total_travel_time, total_FFTT, total_travel_time- total_FFTT,  total_distance);
				}

			}

			//fprintf(logfile_od, "Origin: %d, Destination: %d, Route Flow: %f, tt: %f, distance: %f,FFTT:%f \n", Orig, Dest, RouteFlow,
			//	total_travel_time, total_distance, total_FFTT);
		}
	}


/*	fclose(logfile_od)*/;

	StatusMessage("Assign", "Finished assign.");
}


int get_number_of_nodes_from_node_file(int &number_of_zones, int &l_FirstThruNode)
{
	number_of_zones = 0; 
	CDTACSVParser parser_node;
	l_FirstThruNode = 1;
	int number_of_nodes = 0;

	if (parser_node.OpenCSVFile("node.csv", true))
	{


		while (parser_node.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{


			// Read node id
			int node_id = 0; 
			int zone_id = 0; 
			parser_node.GetValueByFieldName("node_id", node_id);
			parser_node.GetValueByFieldName("zone_id", zone_id);

			g_map_node_seq_no_2_external_node_id[number_of_nodes+1] = node_id;
			g_map_external_node_id_2_node_seq_no[node_id] = number_of_nodes+1;  // this code node sequential number starts from 1 

			if(zone_id >=1 && zone_id > number_of_zones)
				number_of_zones = zone_id;

			if (zone_id == 0 && l_FirstThruNode ==1 /* not initialized*/)
				l_FirstThruNode = node_id;

			number_of_nodes++;

		}

		parser_node.CloseCSVFile();



	}

	return  number_of_nodes;

}

int get_number_of_links_from_link_file()
{

	CDTACSVParser parser_link;

	int number_of_links = 0;

	if (parser_link.OpenCSVFile("link.csv", true))
	{


		while (parser_link.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
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

	return  number_of_links;

}


void main(int argc, char** argv) 
{
	double * MainVolume, * SubVolume, * SDVolume, Lambda;
	int** MinPathPredLink;

	////StatusMessage("General", "Ready, set, go...");
	//switch (argc) {
	//case 2: 	tuiFileName = argv[1];
	//	break;
	////case 1:		tuiFileName = "control.tui";
	////	break;
	//default:	ExitMessage("Wrong number of command line arguments (%d). \n"
	//	"Syntax: fw <text user interface file>.", argc - 1);
	//}

	no_nodes = get_number_of_nodes_from_node_file(no_zones, FirstThruNode);
	no_links = get_number_of_links_from_link_file();


	printf("no_nodes= %d, no_zones = %d, FirstThruNode = %d, no_links = %d\n", no_nodes, no_zones, FirstThruNode, no_links);

	fopen_s(&link_performance_file, "link_performance.csv", "w");
	if (link_performance_file == NULL) {
		printf("Error opening file!\n");
		return;
	}
	fclose(link_performance_file);

	fopen_s(&link_performance_file, "link_performance.csv", "a+");

	AssignIterations = 20;
	fopen_s(&logfile, "TAP_log.csv", "w");  // Open the log file for writing.

	double system_wide_travel_time = 0;
	double system_least_travel_time = 0;

	fprintf(logfile, "iteration_no,link_id,internal_from_node_id,internal_to_node_id,volume,capacity,voc,fftt,travel_time,delay\n");

	Init(no_zones);
	int iteration_no = 0;
	MainVolume = (double*)Alloc_1D(no_links, sizeof(double));
	SDVolume = SubVolume = (double*)Alloc_1D(no_links, sizeof(double)); /* Compute search direction and sub-volume in the same place. */
	MinPathPredLink = (int**)Alloc_2D(no_zones, no_nodes, sizeof(int));


	//InitFWstatus(&fw_status);
	system_wide_travel_time = UpdateLinkCost(MainVolume);  // set up the cost first using FFTT

	fprintf(link_performance_file, "iteration_no,link_id,internal_from_node_id,internal_to_node_id,volume,ref_volume,capacity,voc,fftt,travel_time,delay\n");

	for (int k = 1; k <= no_links; k++)
	{
		fprintf(link_performance_file, "%d,%d,%d,%d,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf\n", iteration_no, k, Link[k].external_from_node_id, Link[k].external_to_node_id, MainVolume[k], Link[k].Capacity, MainVolume[k]/fmax(0.01, Link[k].Capacity), Link[k].FreeTravelTime, Link[k].Travel_time, Link[k].Travel_time - Link[k].FreeTravelTime);
	}

	system_least_travel_time = FindMinCostRoutes(MinPathPredLink);
	Assign(iteration_no, ODflow, MinPathPredLink, MainVolume);
	//FirstFWstatus(MainVolume, &fw_status);
	system_wide_travel_time = UpdateLinkCost(MainVolume);
	double gap = (system_wide_travel_time - system_least_travel_time) / (fmax(0.1, system_least_travel_time)) * 100;

	printf("iter No = %d, sys. TT =  %lf, least TT =  %lf, gap = %f\n", iteration_no, system_wide_travel_time, system_least_travel_time, gap);

	for (int k = 1; k <= no_links; k++)
	{
		fprintf(logfile, "%d,%d,%d,%d,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf\n", iteration_no, k, Link[k].external_from_node_id, Link[k].external_to_node_id, MainVolume[k], Link[k].Ref_volume, Link[k].Capacity, MainVolume[k] / fmax(0.01, Link[k].Capacity), Link[k].FreeTravelTime, Link[k].Travel_time, Link[k].Travel_time - Link[k].FreeTravelTime);
	}


	//for (int k = 1; k <= no_links; k++)
	//{
	//	fprintf(link_performance_file, "%d,%d,%d,%d,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf\n", iteration_no, k, Link[k].internal_from_node_id, Link[k].internal_to_node_id, MainVolume[k], Link[k].Capacity, Link[k].FreeTravelTime, Link[k].Travel_time, Link[k].Travel_time - Link[k].FreeTravelTime);
	//}


	for (iteration_no = 1; iteration_no < AssignIterations; iteration_no++)
	{
		system_least_travel_time = FindMinCostRoutes(MinPathPredLink);
		Assign(iteration_no, ODflow, MinPathPredLink, SubVolume);
		VolumeDifference(SubVolume, MainVolume, SDVolume);	/* Which yields the search direction. */
		Lambda = LinksSDLineSearch(MainVolume, SDVolume);

		// MSA options
		Lambda = 1.0 / (iteration_no + 1); 
		//UpdateFWstatus(MainVolume, SDVolume, &fw_status);
		UpdateVolume(MainVolume, SDVolume, Lambda);
		system_wide_travel_time = UpdateLinkCost(MainVolume);


		gap = (system_wide_travel_time - system_least_travel_time) / (fmax(0.1, system_least_travel_time)) * 100;
		printf("iter No = %d, Lambda = %f, sys. TT =  %lf, least TT =  %lf, gap = %f\n", iteration_no, Lambda, system_wide_travel_time, system_least_travel_time, gap);

		for (int k = 1; k <= no_links; k++)
		{
			fprintf(logfile, "%d,%d,%d,%d,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf\n", iteration_no, k, Link[k].external_from_node_id, Link[k].external_to_node_id, MainVolume[k], Link[k].Ref_volume, Link[k].Capacity, MainVolume[k] / fmax(0.01, Link[k].Capacity), Link[k].FreeTravelTime, Link[k].Travel_time, Link[k].Travel_time - Link[k].FreeTravelTime);
		}

		for (int k = 1; k <= no_links; k++)
		{
			fprintf(link_performance_file, "%d,%d,%d,%d,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf\n", 
				iteration_no, 
				k, 
				Link[k].external_from_node_id,
				Link[k].external_to_node_id,
				MainVolume[k],
				Link[k].Ref_volume,
				Link[k].Capacity, 
				Link[k].FreeTravelTime, 
				Link[k].Travel_time, 
				Link[k].Travel_time - Link[k].FreeTravelTime);
		}
	}

	for (int k = 1; k <= no_links; k++)
	{
		fprintf(logfile, "%d,%d,%d,%d,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf\n", iteration_no, k, Link[k].external_from_node_id, Link[k].external_to_node_id, MainVolume[k], Link[k].Capacity, MainVolume[k] / fmax(0.01, Link[k].Capacity), Link[k].FreeTravelTime, Link[k].Travel_time, Link[k].Travel_time - Link[k].FreeTravelTime);
	}

	for (int k = 1; k <= no_links; k++)
	{
		fprintf(link_performance_file, "%d,%d,%d,%d,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf\n", iteration_no, k, Link[k].external_from_node_id, Link[k].external_to_node_id, MainVolume[k], Link[k].Ref_volume, Link[k].Capacity, Link[k].FreeTravelTime, Link[k].Travel_time, Link[k].Travel_time - Link[k].FreeTravelTime);
	}

	free(MainVolume);
	free(SubVolume);
	Free_2D((void**)MinPathPredLink, no_zones, no_nodes);

	Close();

	fclose(link_performance_file);
	fclose(logfile);  // Close the log file when you're done with it.

}



static void Init(int input_no_zones) {
	//tuiInit(tuiFileName);
	InitLinks();
	InitODflow(input_no_zones);
	InitLineSearch();
}

static void Close() {
	StatusMessage("General", "Closing all modules");
	//tuiClose(tuiFileName);
	CloseLinks();
	CloseODflow();
	CloseLineSearch();
}

static void InitODflow(int input_no_zones) {
	char ODflowFileName[15];

	double Factor = 1.0;

	//tuiGetInputFileName("OD flow file name", TRUE, ODflowFileName);
	StatusMessage("General", "Reading OD flow file");
	ODflow = Read_ODflow(&TotalODflow, &input_no_zones);

}


static void CloseODflow(void) {
	Free_2D((void**)ODflow, no_zones, no_zones);
}


void* Alloc_1D(int dim1, size_t size) {
	void* Array;

	Array = (void*)calloc(dim1 + 1, size);
	if (Array == NULL) {
		ExitMessage("Can not allocate memory for single dimension array of size %d. \n",
			dim1);
	}
	return Array;
}

void** Alloc_2D(int dim1, int dim2, size_t size) {
	void** Array;
	int i;

	Array = (void**)calloc(dim1 + 1, sizeof(void*));
	if (Array == NULL) {
		ExitMessage("Can not allocate memory for two-dimensions array of size %d by %d. \n",
			dim1, dim2);
	}
	for (i = 1; i <= dim1; i++) {
		Array[i] = (void*)calloc(dim2 + 1, size);
		if (Array[i] == NULL) {
			ExitMessage("Can not allocate memory for two-dimensions array of size %d by %d. \n",
				dim1, dim2);
		}
	}
	return (Array);
}

void Free_2D(void** Array, int dim1, int dim2) {
	int i;
	void* p;

	for (i = 1; i <= dim1; i++) {
		p = Array[i];
		free(p);
	}
	free(Array);
}


/* Internal functions */

double Link_Delay(int k, double* Volume)
{
	Link[k].Travel_time = Link[k].FreeTravelTime * (1.0 + Link[k].BoverC * (pow(Volume[k], Link[k].Power)));
	return (Link[k].FreeTravelTime * (1.0 + Link[k].BoverC * (pow(Volume[k], Link[k].Power))));
}

double LinkDelay_Integral(int k, double* Volume) {
	if (Link[k].Power >= 0.0)
		return (Volume[k] * Link[k].FreeTravelTime * (1.0 + (Link[k].BoverC / (Link[k].Power + 1)) * pow(Volume[k], Link[k].Power)));
	else
		return 0.0;
}

double Link_Delay_Der(int k, double* Volume) {
	if (Link[k].Power == 0.0)
		return 0.0;
	else
		return (Link[k].FreeTravelTime * Link[k].BoverC * Link[k].Power * pow(Volume[k], (Link[k].Power - 1)));
}


double AdditionalCost(int k) {
	double AddCost = 0;

	//AddCost = AutoCostCoef[DIST] * Link[k].Distance +
	//	+AutoCostCoef[MONETARY] * Link[k].Toll;

	return  AddCost;
}




double Link_GenCost(int k, double* Volume) {
	//	return (Link[k].AdditionalCost + Link_Delay(k, Volume));
	return (Link_Delay(k, Volume));
}

double LinkCost_Integral(int k, double* Volume) {

	return (Link[k].AdditionalCost * Volume[k] + LinkDelay_Integral(k, Volume));
}

double Link_GenCostDer(int k, double* Volume) {

	return (Link_Delay_Der(k, Volume));
}




/* External functions */

void ClearVolume(double* VolumeArray) {
	int k;
	for (k = 1; k <= no_links; k++)
		VolumeArray[k] = 0.0;
}

void VolumeDifference(double* Volume1, double* Volume2, double* Difference) {
	int k;
	for (k = 1; k <= no_links; k++)
		Difference[k] = Volume1[k] - Volume2[k];

}


void UpdateVolume(double* MainVolume, double* SDVolume, double Lambda) {
	int k;
	for (k = 1; k <= no_links; k++) {
		MainVolume[k] += Lambda * SDVolume[k];
	}
}

void UpdateLinkAdditionalCost(void) {
	int k;

	for (k = 1; k <= no_links; k++)
		Link[k].AdditionalCost = AdditionalCost(k);
}

double UpdateLinkCost(double* MainVolume) {
	int k;
	double system_wide_travel_time = 0;

	for (k = 1; k <= no_links; k++)
	{
		Link[k].Delay = Link_Delay(k, MainVolume);
		Link[k].Travel_time = Link_Delay(k, MainVolume);

		Link[k].GenCost = Link_GenCost(k, MainVolume);
		system_wide_travel_time += (MainVolume[k] * Link[k].Delay);
	}

	return system_wide_travel_time;
}

void UpdateLinkCostDer(double* MainVolume)
{
	int k;

	for (k = 1; k <= no_links; k++) {
		Link[k].GenCostDer = Link_GenCostDer(k, MainVolume);
	}
}


void GetLinkTravelTimes(double* Volume, double* TravelTime) {
	int k;

	for (k = 1; k <= no_links; k++) {
		TravelTime[k] = Link_Delay(k, Volume);
	}
}

double TotalLinkCost(double* Volume) {
	int k;
	double Sum = 0;

	for (k = 1; k <= no_links; k++)
		Sum += Link[k].GenCost * Volume[k];
	return Sum;
}



double OFscale = 1.0;


double OF_Links(double* MainVolume) {
	int k;
	double Sum = 0;

	for (k = 1; k <= no_links; k++)
		Sum += LinkCost_Integral(k, MainVolume);

	return Sum / OFscale;
}



double OF_LinksDirectionalDerivative(double* MainVolume, double* SDVolume, double Lambda) {
	int k;
	double* Volume;
	double LinkCostSum = 0;

	Volume = (double*)Alloc_1D(no_links, sizeof(double));

	for (k = 1; k <= no_links; k++) {
		Volume[k] = MainVolume[k] + Lambda * SDVolume[k];
	}
	for (k = 1; k <= no_links; k++) {
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
	std::vector <CLink> links;

	std::string line;



	if (parser_link.OpenCSVFile("link.csv", true))
	{

		int line_no = 0;

		int k = 1; // link start from 1  

		while (parser_link.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{

			std::string value;
			//CLink link;
			int lanes = 0;
			float capacity = 0;
			float free_speed = 10;
			// Read link_id
			parser_link.GetValueByFieldName("from_node_id", Link[k].external_from_node_id);
			parser_link.GetValueByFieldName("to_node_id", Link[k].external_to_node_id);

			Link[k].internal_from_node_id = Link[k].external_from_node_id;
			Link[k].internal_to_node_id = Link[k].external_to_node_id;

			if(g_map_external_node_id_2_node_seq_no.find(Link[k].external_from_node_id)!= g_map_external_node_id_2_node_seq_no.end())
						Link[k].internal_from_node_id = g_map_external_node_id_2_node_seq_no[Link[k].external_from_node_id];

			if (g_map_external_node_id_2_node_seq_no.find(Link[k].external_to_node_id) != g_map_external_node_id_2_node_seq_no.end())
				Link[k].internal_to_node_id = g_map_external_node_id_2_node_seq_no[Link[k].external_to_node_id];




			parser_link.GetValueByFieldName("length", Link[k].Distance);
			parser_link.GetValueByFieldName("ref_volume", Link[k].Ref_volume);

			
			parser_link.GetValueByFieldName("lanes", lanes);
			parser_link.GetValueByFieldName("capacity", capacity);
			Link[k].Capacity = lanes * capacity;

			parser_link.GetValueByFieldName("free_speed", free_speed);

			// Read internal_from_node_id

			// Read length



			// Read capacity





			Link[k].FreeTravelTime = Link[k].Distance / free_speed * 60.0;


			parser_link.GetValueByFieldName("VDF_alpha", Link[k].B);
			parser_link.GetValueByFieldName("VDF_beta", Link[k].Power);

			if (capacity > 0)
				Link[k].BoverC = Link[k].B / pow(capacity, Link[k].Power);
			else 
				Link[k].BoverC = 0;
				

				k++;
		}

			parser_link.CloseCSVFile();
	}



		//int count, k, internal_from_node_id, internal_to_node_id, Type;
		//double Capacity, Distance, FreeTravelTime, B, Power, Speed, Toll;
		//const char* LinkArgumentDescription[10] = { "internal_from_node_id","head","capacity","length",
		//	"free flow travel time","power","speed limit","toll","link type","semicolomn - for end of link" };
		///* Descriptions of the various link record arguments, used for error messages. */

		//FILE* LinksFile2;
		//errno_t err = fopen_s(&LinksFile2, "link.csv", "r");
		//if (err) {
		//	printf("Failed to open file\n");
		//	return;
		//}

		//for (k = 1; ; k++)
		//{ // Loop until break
		//	count = fscanf_s(LinksFile2, "%d %d %lf %lf %lf %lf %lf %lf %lf %d %1[;]",
		//		&internal_from_node_id, &internal_to_node_id, &Capacity, &Distance, &FreeTravelTime,
		//		&B, &Power, &Speed, &Toll, &Type, Semicolon, sizeof(Semicolon));
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


		//	//if (Power < 0)
		//	//{
		//	//std::cout <<	("Link file '%s', link %d power %lf < 0 is decreasing cost",
		//	//		LinksFileName, k, Link[k].Power);
		//	//}

		//	if (Capacity > 0)
		//		Link[k].BoverC = B / pow(Capacity, Power);
		//	else {
		//		Link[k].BoverC = 0;
		//		//InputWarning("link file '%s', link %d from %d to %d has %lf capacity \n",
		//		//	LinksFileName, k, internal_from_node_id, internal_to_node_id, Capacity);
		//	}

		//}

		//fclose(LinksFile2);

}


static void InitLinkPointers(char* LinksFileName) {
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
		else if (internal_from_node_id == Node + 1) {
			LastLinkFrom[Node] = k - 1;
			Node = internal_from_node_id;
			FirstLinkFrom[Node] = k;
		}

		/**********************
		CHECKING FOR SORT ERRORS AND GAPS IN THE LINKS FILE.
		*********************/
		else if (internal_from_node_id < Node)
		{
			//ExitMessage("Sort error in link file '%s': a link from node %d was found after "
			//	"a link from node %d \n", LinksFileName, internal_from_node_id, Node);
		}
		else if (internal_from_node_id > Node + 1) {
			//InputWarning("link file '%s' has no links out from "
			//	"nodes %d through %d. \n", LinksFileName, Node + 1, internal_from_node_id - 1);
			LastLinkFrom[Node] = k - 1;
			for (Node++; Node < internal_from_node_id; Node++) {
				FirstLinkFrom[Node] = 0;
				LastLinkFrom[Node] = -1;
			}
			FirstLinkFrom[Node] = k; /* Node equals internal_from_node_id now. */
		}
	}

	if (Node == no_nodes) {
		LastLinkFrom[Node] = no_links;	/* Now Node equals no_nodes in any case */
	}
	else {
		//InputWarning("link file '%s' has no links out from "
		//	"nodes %d through %d. \n", LinksFileName, Node + 1, no_nodes);
		LastLinkFrom[Node] = k - 1;
		for (Node++; Node <= no_nodes; Node++) {
			FirstLinkFrom[Node] = 0;
			LastLinkFrom[Node] = -1;
		}
	}

}


void FindLinksTo(void) {
	int Node, k;

	LinksTo = (sorted_list*)Alloc_1D(no_nodes, sizeof(sorted_list*));
	for (Node = 1; Node <= no_nodes; Node++)
		LinksTo[Node] = NULL;

	for (k = 1; k <= no_links; k++)
		ListAdd(k, &( LinksTo[Link[k].internal_to_node_id]));
}
void InitLinks(void)
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

void CloseLinks(void) {
	int Node;

	free(Link);
	free(FirstLinkFrom);
	free(LastLinkFrom);
	for (Node = 1; Node <= no_nodes; Node++)
		ListFree(LinksTo[Node]);
	free(LinksTo);


}






/* Report functions */



#define MAX_NO_BISECTITERATION 1000  /* Avoids inifinite loops in the second part */


static int MinLineSearchIterations = 1;
static int ActualIterations = 0;
static double LastLambda = 1.0;


void InitLineSearch(void) {
	//tuiGetInt("Line search iterations {LSITE}", FALSE, &MinLineSearchIterations);
}

void CloseLineSearch(void) {}


double LinksSDLineSearch(double* MainVolume, double* SDVolume) {
	int n;
	double lambdaleft, lambdaright, lambda;
	double grad;


	grad = OF_LinksDirectionalDerivative(MainVolume, SDVolume, 0.0);
	StatusMessage("Line search step", "0.0");
	StatusMessage("Line search grad", "%lf", grad);
	if (grad >= 0) {
		LastLambda = 0.0;
		return(0.0);
	}

	grad = OF_LinksDirectionalDerivative(MainVolume, SDVolume, 1.0);
	StatusMessage("Line search step", "1.0");
	StatusMessage("Line search grad", "%lf", grad);
	if (grad <= 0) {
		LastLambda = 1.0;
		return(1.0);
	}

	lambdaleft = 0;
	lambdaright = 1;
	lambda = 0.5;

	for (n = 1; n <= MinLineSearchIterations; n++) {
		grad = OF_LinksDirectionalDerivative(MainVolume, SDVolume, lambda);
		StatusMessage("Line search step", "%lf", lambda);
		StatusMessage("Line search grad", "%lf", grad);

		if (grad <= 0.0)
			lambdaleft = lambda;
		else
			lambdaright = lambda;

		lambda = 0.5 * (lambdaleft + lambdaright);
	}
	for (; ((lambdaleft == 0) && (n <= MAX_NO_BISECTITERATION)); n++) {
		grad = OF_LinksDirectionalDerivative(MainVolume, SDVolume, lambda);
		StatusMessage("Line search step", "%lf", lambda);
		StatusMessage("Line search grad", "%lf", grad);

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


void print_ls_header(FILE* fp) {
	fprintf(fp, "LS iterations \tStep size \t");
}

void ls_report(FILE* fp) {
	fprintf(fp, "%d \t%g \t", ActualIterations, LastLambda);
}


#define BUFFERSIZE 100







static struct list_item* NewListItem(int value) {
	struct list_item* new_item;

	new_item = (struct list_item*)malloc(sizeof(struct list_item));
	if (new_item == NULL) {
		printf("Error in memory allocation of list item. \n");
		exit(EXIT_FAILURE);
	}
	(*new_item).value = value;
	(*new_item).next_item = NULL;

	return(new_item);
}



/* Check if value is in list */
int ListFind(int Value, sorted_list list) {
	struct list_item* item;

	for (item = list; item != NULL && (*item).value < Value; item = (*item).next_item);
	if (item != NULL && (*item).value == Value)
		return 1;
	else
		return 0;
}




/* Try to add a new item to the list.
It is assumed that value is greater then the first value in the list.
Return the list item that contains "value".*/

static struct list_item* InListAdd(int value, struct list_item* item) {
	struct list_item* new_item, * remaining_list;

	/* If there are no more items in the list, add the new value in the end. */
	if ((*item).next_item == NULL) {
		new_item = NewListItem(value);
		(*item).next_item = new_item;
		return(new_item);
	}

	remaining_list = (*item).next_item;

	if (value == (*remaining_list).value) {
		return(remaining_list);
	}

	else if (value < (*remaining_list).value) {
		new_item = NewListItem(value);
		(*new_item).next_item = remaining_list;
		(*item).next_item = new_item;
		return(new_item);
	}


	else {
		return(InListAdd(value, remaining_list));
	}

}


void ListAdd(int value, sorted_list* list) {
	struct list_item* new_item;

	if (*list == NULL) {
		*list = NewListItem(value);
	}
	/* Adding the value in the beginning is a special case, all others are handled by recursion. */
	else if (value < (**list).value) {
		new_item = NewListItem(value);
		(*new_item).next_item = (sorted_list)*list;
		*list = new_item;
	}
	else if (value > (**list).value) {
		InListAdd(value, *list);
	}

	/* If the new value is equal to the first value then there is nothing to do. */

}


/* Try to remove an item from a list.
It is assumed that value is greater then the first value in the list.
If value is found in the list, return 1 and remove it;
otherwise return 0 and do not change the list. */

static int InListRemove(int value, struct list_item* item) {
	struct list_item* remaining_list;

	if ((*item).next_item == NULL)
		return(0);

	remaining_list = (*item).next_item;
	if (value == (*remaining_list).value) {
		(*item).next_item = (*remaining_list).next_item;
		free(remaining_list);
		return(1);
	}
	else if (value < (*remaining_list).value) {
		return(0);
	}
	else {
		return(InListRemove(value, remaining_list));
	}
}

int ListRemove(int value, sorted_list* list) {
	struct list_item* item_to_free;

	if (*list == NULL) {
		return(0);
	}
	else if (value < (**list).value) {
		return(0);
	}
	else if (value == (**list).value) {
		item_to_free = (sorted_list)*list;
		*list = (**list).next_item;
		free(item_to_free);
		return(1);
	}
	else {		/* value > (**list).value */
		return(InListRemove(value, *list));
	}
}


static void InListFree(struct list_item* item) {

	if ((*item).next_item != NULL)
		InListFree((*item).next_item);
	free(item);
}

void ListFree(sorted_list list) {

	if (list != NULL)
		InListFree(list);
}



/* Creat a copy of a list. */
sorted_list ListCopy(sorted_list list) {
	sorted_list new_list;
	struct list_item* item, * new_item;

	/* If the first list is empty, return an empty list. */
	if (list == NULL)
		return(NULL);

	else {
		item = list;
		new_list = new_item = NewListItem((*item).value);
		item = (*item).next_item;
		while (item != NULL) {
			(*new_item).next_item = NewListItem((*item).value);
			new_item = (*new_item).next_item;
			item = (*item).next_item;
		}
		return(new_list);
	}
}


/* Merge list1 into list2. */
void ListMerge(sorted_list list1, sorted_list* list2) {
	struct list_item* item1, * item2;

	/* If the first list is empty, there is nothing to do. */
	if (list1 == NULL)
		return;

	/* If the second list empty, set it to a copy of list1. */
	if (*list2 == NULL) {
		*list2 = ListCopy(list1);
	}

	else {
		/* InListAdd assumes that the first value in the list is lower then the new value. */
		/* Therefore, if the first value in list1 is smaller, make it the first value in list2. Other values wil be larger. */
		if ((*list1).value < (**list2).value) {
			item2 = NewListItem((*list1).value);
			(*item2).next_item = (sorted_list)*list2;
			*list2 = item2;
			item1 = (*list1).next_item;
		}
		/* if the first value in list1 and list2 are equal, skip the first value in list1. Other values wil be larger. */
		else if ((*list1).value == (**list2).value) {
			item2 = (sorted_list)*list2;
			item1 = (*list1).next_item;
		}
		/* Otherwise, if the first value in list1 is greater, start from the first value of list1. */
		else {
			item2 = (sorted_list)*list2;
			item1 = list1;
		}


		/* Add the remaining items of list1 one by one, continue always from the last item added.
		(previous items have lower values then the remaining values of list1.) */
		for (; item1 != NULL; item1 = (*item1).next_item)
			item2 = InListAdd((*item1).value, item2);
	}
}




sorted_list ListIntersect(sorted_list list1, sorted_list list2) {
	struct list_item* item1, * item2;
	sorted_list new_list = NULL;

	item1 = list1;
	item2 = list2;
	while (item1 != NULL && item2 != NULL) {
		if ((*item1).value < (*item2).value)
			item1 = (*item1).next_item;
		else if ((*item2).value < (*item1).value)
			item2 = (*item2).next_item;
		else {		/* (*item1).value==(*item2).value */
			ListAdd((*item1).value, &new_list);
			item1 = (*item1).next_item;
			item2 = (*item2).next_item;
		}
	}
	return(new_list);
}





/* Find all elements in list1 that do not show up in list2. */
sorted_list ListDifference(sorted_list list1, sorted_list list2) {
	struct list_item* item1, * item2;
	sorted_list new_list = NULL;

	item1 = list1;
	item2 = list2;
	while (item1 != NULL && item2 != NULL) {
		if ((*item1).value < (*item2).value) {
			ListAdd((*item1).value, &new_list);
			item1 = (*item1).next_item;
		}
		else if ((*item2).value < (*item1).value) {
			item2 = (*item2).next_item;
		}
		else {			/* (*item1).value = (*item2).value */
			item1 = (*item1).next_item;
			item2 = (*item2).next_item;
		}
	}

	while (item1 != NULL) { 		/* Add remaining items of list1, if there are any. */
		ListAdd((*item1).value, &new_list);
		item1 = (*item1).next_item;
	}

	return(new_list);
}



int ListsAreEqual(sorted_list list1, sorted_list list2) {
	struct list_item* item1, * item2;

	for (item1 = list1, item2 = list2; item1 != NULL; item1 = (*item1).next_item) {
		if (item2 == NULL)
			return(0);
		else if ((*item1).value != (*item2).value)
			return(0);
		item2 = (*item2).next_item;
	}
	if (item2 != NULL)
		return(0);
	else
		return(1);
}


int ListSize(sorted_list list) {
	struct list_item* item;
	int i;

	for (i = 0, item = list; item != NULL; i++, item = (*item).next_item);

	return(i);
}


void StatusMessage(const char* group, const char* format, ...) {
	double new_time;


}


void Read_ODtable(double** ODtable, int no_zones) 
{
	char ch, Coloumn[2], Semicoloumn[2]; /* Reserve room for the '\0' in the fscanf_s. */
	int Orig, Dest, NewOrig, NewDest;
	double Value;

	FILE* file;
	fopen_s(&file, "demand.csv", "r");

	if (file == NULL) {
		printf("Failed to open file\n");
		return;
	}

	int o_zone_id, d_zone_id;
	double volume;

	// Skip the header line
	char header[100];
	if (fgets(header, sizeof(header), file) == NULL) {
		printf("Failed to read header\n");
		return;
	}

	int line_count = 0;
	double total_volume = 0;
	// Read the data
	int result;
	while ((result = fscanf_s(file, "%d,%d,%lf", &o_zone_id, &d_zone_id, &volume)) != EOF)
	{
		if (result == 3) // we have read all the 3 values correctly
		{
			if (line_count <= 3)
			{
				printf("o_zone_id: %d, d_zone_id: %d, volume: %.2lf\n", o_zone_id, d_zone_id, volume);
				line_count++;
			}
			ODtable[o_zone_id][d_zone_id] = volume;
			total_volume += volume;
		}
		else
		{
			printf("Error reading line\n");
			break;
		}
	}

	printf("total_volume = %f\n", total_volume);


	fclose(file);

}

double Sum_ODtable(double** ODtable, int no_zones) {
	int Orig, Dest;
	double sum = 0.0;

	for (Orig = 1; Orig <= no_zones; Orig++)
		for (Dest = 1; Dest <= no_zones; Dest++)
			sum += ODtable[Orig][Dest];
	return(sum);
}

double** Read_ODflow(double* TotalODflow, int* no_zones) {
	FILE* ODflowFile;
	double RealTotal, InputTotal;

	ODflow = (double**)Alloc_2D(*no_zones, *no_zones, sizeof(double));
	RouteCost = (double**)Alloc_2D(*no_zones, *no_zones, sizeof(double));
	Read_ODtable(ODflow, *no_zones);

	RealTotal = (double)Sum_ODtable(ODflow, *no_zones);

	*TotalODflow = (double)RealTotal;

	return(ODflow);
}


void ExitMessage(const char* format, ...) {
	va_list ap;

		//vprintf(format, ap);
	printf("\n");

	getchar();

	exit(EXIT_FAILURE);
}