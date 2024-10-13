#pragma once


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
