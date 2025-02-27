#include "cre_ALF.h"

namespace
{
    const int kPort = 7001;

    // environment setup
    //default values, if you want modify these parameters check SetupVirtualEnvironments()
    double vArena_size = 2.0;
    const double kScaling = 1.0;
    const double kKiloDiameter = 0.034;
    double vDistance_threshold = vArena_size / 2.0 - 2.0 * kKiloDiameter;
    const double kM_TO_CM = 100.0;
    const double kCLIENT_SERVER_SCALE = 4.0;

    // wall avoidance stuff
    const CVector2 up_direction(0.0, -1.0);
    const CVector2 down_direction(0.0, 1.0);
    const CVector2 left_direction(1.0, 0.0);
    const CVector2 right_direction(-1.0, 0.0);
    int proximity_bits = 8;

    //counter for LOG files, incremented each second
    int internal_counter = 0;

    //decision message (for flying robots) bits in decimal
    const double kResourceNorthBits = 63.0;
    const double kResourceSouthBits = 63.0;
}
CALFClientServer::CALFClientServer() : m_unDataAcquisitionFrequency(20)
{
}

CALFClientServer::~CALFClientServer()
{
}

void CALFClientServer::Init(TConfigurationNode &t_node)
{
    CALF::Init(t_node);

    /* Read parameters */
    TConfigurationNode &tModeNode = GetNode(t_node, "extra_parameters");
    GetNodeAttribute(tModeNode, "mode", mode);
    GetNodeAttribute(tModeNode, "ip_addr", IP_ADDR);
    random_seed = GetSimulator().GetRandomSeed();
    // GetNodeAttribute(tModeNode, "random_seed", random_seed);
    if (mode == "CLIENT")
    {
        GetNodeAttribute(tModeNode, "desired_North_areas", desired_North_areas);
        GetNodeAttribute(tModeNode, "desired_South_areas", desired_South_areas);
    }
    GetNodeAttribute(tModeNode, "communication_range", communication_range);

    /*********** LOG FILES *********/
    m_kiloOutput.open(m_strKiloOutputFileName, std::ios_base::trunc | std::ios_base::out);
    m_taskOutput.open(m_strTaskOutputFileName, std::ios_base::trunc | std::ios_base::out);

    /*******************************/

    if (mode == "SERVER")
    {
        GetNodeAttribute(tModeNode, "vision_range", vision_range);
    }
    /* Randomly select the desired number of tasks between the available ones, set color and communicate them to the server */
    if (mode == "CLIENT")
    {
        srand(random_seed);

        /* GENERATE RANDOM TASK for NORTH and SOUTH */
        std::default_random_engine re;
        re.seed(random_seed);

        /* Active IDs */
        while (activated_North_areas.size() < desired_North_areas)
        {
            if (desired_North_areas - 1 > max_North_area_id)
            {
                std::cerr << "Requested more areas then the available ones, WARNING!";
            }
            std::uniform_int_distribution<int> distr(0, max_North_area_id);
            int random_number;
            do
            {
                random_number = distr(re);
                //std::cout<<"actRed"<<activated_North_areas.size()<<" added:"<<random_number<<std::endl;
            } while (std::find(activated_North_areas.begin(), activated_North_areas.end(), random_number) != activated_North_areas.end());
            activated_North_areas.push_back(random_number);
            multiArea[random_number].Completed = false;
        }
        std::sort(activated_North_areas.begin(), activated_North_areas.end());

        while (activated_South_areas.size() < desired_South_areas)
        {
            if (desired_South_areas - 1 > max_South_area_id)
            {
                std::cerr << "Requested more areas then the available ones, WARNING!";
            }
            std::uniform_int_distribution<int> distr(max_North_area_id + 1, max_South_area_id);
            int random_number;
            do
            {
                random_number = distr(re);
                //std::cout << "actRed" << activated_South_areas.size() << " added:" << random_number << std::endl;
            } while (std::find(activated_South_areas.begin(), activated_South_areas.end(), random_number) != activated_South_areas.end());
            activated_South_areas.push_back(random_number);
            multiArea[random_number].Completed = false;
        }
        std::sort(activated_South_areas.begin(), activated_South_areas.end());
    }

    /* Initializations */
    bytesReceived = -1;
    memset(storeBuffer, 0, 2000);
    outputBuffer = "";

    /* Opening communication port */
    serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    std::string ipAddress = IP_ADDR;
    sockaddr_in hint;
    hint.sin_family = AF_INET;
    hint.sin_port = htons(kPort);
    inet_pton(AF_INET, ipAddress.c_str(), &hint.sin_addr);

    if (mode == "SERVER")
    {
        bind(serverSocket, (sockaddr *)&hint, sizeof(hint));
        listen(serverSocket, SOMAXCONN);
        sockaddr_in client;
        socklen_t clientSize = sizeof(client);
        clientSocket = accept(serverSocket, (sockaddr *)&client, &clientSize);
        char host[NI_MAXHOST];
        char service[NI_MAXSERV];
        memset(host, 0, NI_MAXHOST);
        memset(service, 0, NI_MAXSERV);
        if (getnameinfo((sockaddr *)&client, sizeof(client), host, NI_MAXHOST, service, NI_MAXSERV, 0) == 0)
        {
            std::cout << host << " connected on port " << service << std::endl;
        }
        else
        {
            inet_ntop(AF_INET, &client.sin_addr, host, NI_MAXHOST);
            std::cout << host << " connected on port " << ntohs(client.sin_port) << std::endl;
        }
        close(serverSocket);
    }
    if (mode == "CLIENT")
    {
        int conn = -1;
        while (conn != 0)
        {
            conn = connect(serverSocket, (sockaddr *)&hint, sizeof(hint));
            std::cout << "CONNECTION VALUE: " << conn << std::endl;
        }
    }
}

void CALFClientServer::Reset()
{
    m_kiloOutput.close();
    m_kiloOutput.open(m_strKiloOutputFileName, std::ios_base::trunc | std::ios_base::out);
    if (mode == "CLIENT")
    {
        m_taskOutput.close();
        m_taskOutput.open(m_strTaskOutputFileName, std::ios_base::trunc | std::ios_base::out);
    }
}

void CALFClientServer::Destroy()
{
    m_kiloOutput.close();

    if (mode == "CLIENT")
    {
        m_taskOutput.close();
        close(serverSocket);
    }
    if (mode == "SERVER")
    {
        close(clientSocket);
    }
}

void CALFClientServer::SetupInitialKilobotStates()
{
    m_vecKilobotStates_ALF.resize(m_tKilobotEntities.size());
    m_vecKilobotsPositions.resize(m_tKilobotEntities.size());
    m_vecKilobotsOrientations.resize(m_tKilobotEntities.size());
    m_vecLastTimeMessaged.resize(m_tKilobotEntities.size());
    m_fMinTimeBetweenTwoMsg = Max<Real>(1.0, m_tKilobotEntities.size() * m_fTimeForAMessage / 3.0);

    for (UInt16 it = 0; it < m_tKilobotEntities.size(); it++)
    {
        SetupInitialKilobotState(*m_tKilobotEntities[it]);
    }

    /* Initialization of kilobots variables */
    actual_orientation = std::vector<int>(m_tKilobotEntities.size(), 0);
    command = std::vector<int>(m_tKilobotEntities.size(), 0);
}

void CALFClientServer::SetupInitialKilobotState(CKilobotEntity &c_kilobot_entity)
{
    UInt16 unKilobotID = GetKilobotId(c_kilobot_entity);
    m_vecLastTimeMessaged[unKilobotID] = -1;
    m_vecKilobotsPositions[unKilobotID] = GetKilobotPosition(c_kilobot_entity);
    m_vecKilobotsOrientations[unKilobotID] = GetKilobotOrientation(c_kilobot_entity);
    m_vecKilobotStates_ALF[unKilobotID] = UNCOMMITTED;
}

void CALFClientServer::SetupVirtualEnvironments(TConfigurationNode &t_tree)
{
    /* Read arena parameters */
    vArena_size = argos::CSimulator::GetInstance().GetSpace().GetArenaSize().GetX();
    vDistance_threshold = vArena_size / 2.0 - 2.0 * kKiloDiameter;

    TConfigurationNode &tVirtualEnvironmentsNode = GetNode(t_tree, "environments");
    TConfigurationNodeIterator itAct;

    /* Compute number of areas on the field*/
    for (itAct = itAct.begin(&tVirtualEnvironmentsNode); itAct != itAct.end(); ++itAct)
    {
        num_of_areas += 1;
    }

    /* Build the structure with areas data */
    multiArea.resize(num_of_areas);
    int i = 0;
    for (itAct = itAct.begin(&tVirtualEnvironmentsNode); itAct != itAct.end(); ++itAct, i++)
    {
        std::string s = std::to_string(i);
        std::string var = std::string("Area") + std::string(s);
        TConfigurationNode &t_VirtualClusteringHubNode = GetNode(tVirtualEnvironmentsNode, var);
        GetNodeAttribute(t_VirtualClusteringHubNode, "position", multiArea[i].Center);
        GetNodeAttribute(t_VirtualClusteringHubNode, "radius", multiArea[i].Radius);
        //GetNodeAttribute(t_VirtualClusteringHubNode, "color", multiArea[i].Color);    // use this to read areas color from .argos
    }
    /* Green set as default color, then some of the areas turn red */
    for (int ai = 0; ai < multiArea.size(); ai++)
    {
        multiArea[ai].Completed = true;
        if (multiArea[ai].Center.GetY() < 0)
        {
            multiArea[ai].Color = argos::CColor::GREEN;
        }
        else
        {
            multiArea[ai].Color = argos::CColor::RED;
        }
    }
}

void CALFClientServer::GetExperimentVariables(TConfigurationNode &t_tree)
{
    TConfigurationNode &tExperimentVariablesNode = GetNode(t_tree, "variables");
    GetNodeAttribute(tExperimentVariablesNode, "kilo_filename", m_strKiloOutputFileName);
    GetNodeAttribute(tExperimentVariablesNode, "task_filename", m_strTaskOutputFileName);
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "dataacquisitionfrequency", m_unDataAcquisitionFrequency, m_unDataAcquisitionFrequency);
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "m_unEnvironmentPlotUpdateFrequency", m_unEnvironmentPlotUpdateFrequency, m_unEnvironmentPlotUpdateFrequency);
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "timeforonemessage", m_fTimeForAMessage, m_fTimeForAMessage);
}

void CALFClientServer::UpdateKilobotStates()
{

    for (UInt16 it = 0; it < m_tKilobotEntities.size(); it++)
    {
        UpdateKilobotState(*m_tKilobotEntities[it]);
    }

    /* --------- CLIENT --------- */
    if (mode == "CLIENT")
    {
        /* Build the message for the other ALF */
        outputBuffer = "";
        for (int k = 0; k < multiArea.size(); k++)
        {
            if (multiArea[k].Completed == true)
            {
                outputBuffer.append("1");
            }
            else
            {
                outputBuffer.append("0");
            }
        }
        // std::cout << "Sending " << outputBuffer << std::endl;
    }

    /* Send the message to the other ALF*/
    if (mode == "SERVER")
    {
        send(clientSocket, outputBuffer.c_str(), outputBuffer.size() + 1, 0);
    }
    if (mode == "CLIENT")
    {
        send(serverSocket, outputBuffer.c_str(), outputBuffer.size() + 1, 0);
    }
    //std::cout<<"pos and commit:\t"<< outputBuffer << std::endl;
    outputBuffer = "";
}

void CALFClientServer::UpdateKilobotState(CKilobotEntity &c_kilobot_entity)
{
    UInt16 unKilobotID = GetKilobotId(c_kilobot_entity);
    m_vecKilobotsPositions[unKilobotID] = GetKilobotPosition(c_kilobot_entity);
    m_vecKilobotsOrientations[unKilobotID] = GetKilobotOrientation(c_kilobot_entity);

    /* Listen for the other ALF communication */
    memset(inputBuffer, 0, 2000);
    if (mode == "SERVER")
    {
        bytesReceived = recv(clientSocket, inputBuffer, 2000, MSG_DONTWAIT);
    }
    if (mode == "CLIENT")
    {
        bytesReceived = recv(serverSocket, inputBuffer, 2000, MSG_DONTWAIT);
    }
    if ((bytesReceived == -1) || (bytesReceived == 0))
    {
        //std::cout << "not receiving" << std::endl;
    }
    else
    {
        /* Save the received string in a vector, for having data available until next message comes */
        for (int i = 0; i < 2000; i++)
        {
            storeBuffer[i] = inputBuffer[i];
        }
        // std::cout << "inputBuffer" << inputBuffer << std::endl;
        // std::cout << "storeBuffer" << storeBuffer << std::endl;
    }

    /* --------- SERVER --------- */
    if (mode == "SERVER")
    {
        /* Align to server arena */
        for (int a = 0; a < num_of_areas; a++)
        {
            if (storeBuffer[a] - 48 == 0)
            {
                multiArea[a].Completed = false;
            }
            else
            {
                multiArea[a].Completed = true;
            }
        }
    }

    /* --------- CLIENT --------- */
    if (mode == "CLIENT")
    {
        size_t flying_robot_num = strlen(storeBuffer) / 5;
        multiTransmittingKilobot.clear();
        multiTransmittingKilobot.resize(flying_robot_num);
        //for each flying robot storebuffer contains 5 char organised in this way: aabbc
        //aa is the xcord
        //bb is the ycord
        //c is the commit of the flying robot
        //std::cout<<(int)(m_vecKilobotsOrientations[unKilobotID].GetValue()*10)<<std::endl;

        command[unKilobotID] = 0;
        for (int i = 0, j = 0; i < flying_robot_num; i++, j += 5) // 20 è il numero di robot server (flying)
        // TODO : invece di j è semplicemente i che va incrementato di 5
        {
            //acquire data of flying robots
            multiTransmittingKilobot[i].xCoord = (10 * (storeBuffer[j] - 48)) + storeBuffer[j + 1] - 48; //-48 -> ASCII from char to int
            multiTransmittingKilobot[i].yCoord = (10 * (storeBuffer[j + 2] - 48)) + storeBuffer[j + 3] - 48;
            multiTransmittingKilobot[i].commit = storeBuffer[j + 4] - 48;
            //check command from flying robots
            //if zero skip the robot, else chek position
            if (multiTransmittingKilobot[i].commit == 0)
            {
                continue;
            }
            else
            {
                // TODO : remove 25 and use variables M_TO_CM=100 and SCALING=4 --> M_TO_CM / SCALING = 25
                // check if a ground robot is under the cone of transmission of a flying robot
                float xdisp = multiTransmittingKilobot[i].xCoord - ((kM_TO_CM / kCLIENT_SERVER_SCALE) * (m_vecKilobotsPositions[unKilobotID].GetX() + vArena_size / 2.0)); //25* perchè moltiplico per 100 per considerare solo 2 decimali, poi divido per 4 per allineare le arene (una quadrupla dell'altra)
                //+1 per la traslazione, in modo da non avere il centro nel centro dell'arena ma nell'angolino (1 perchè l'arena è larga 2) | 1 = arena_size/2.0
                // il 25 è *100 per prendere i decimali della posizione /4 perchè un'arena è il quadruplo dell'altra
                float ydisp = multiTransmittingKilobot[i].yCoord - ((kM_TO_CM / kCLIENT_SERVER_SCALE) * (m_vecKilobotsPositions[unKilobotID].GetY() + vArena_size / 2.0)); //+1 perchè coordinate traslate nell'origne prima di essere trasmesse, devo traslare anche queste
                float displacement = sqrt((xdisp * xdisp) + (ydisp * ydisp));
                if (displacement < communication_range)
                {
                    //check if the flying robot is in the semiplane opposit to its commitment
                    if (multiTransmittingKilobot[i].yCoord <= (kM_TO_CM / kCLIENT_SERVER_SCALE))
                    {
                        if (multiTransmittingKilobot[i].commit == 1)
                        {
                            command[unKilobotID] = 1; //robot at south, committed for north
                        }
                    }
                    else
                    {
                        if (multiTransmittingKilobot[i].commit == 2)
                        {
                            command[unKilobotID] = 2; //robot at north, committed for south
                        }
                    }
                }
            }
        }
    }

    /* --------- SERVER --------- */
    //parte speak, compone il messaggio da mandare all'altro ark
    if (mode == "SERVER")
    {
        /* Send position of each robot and the chosen direction */
        /* Transformation for expressing coordinates in 4 characters: origin translated to bottom right corner to have only positive values, then get first 2 digit after the comma */
        std::string pos = std::to_string(m_vecKilobotsPositions[unKilobotID].GetX() + (vArena_size / 2.0)); //posizione del ground robot e la trasla in modo da mandare solo positivi
        std::string pos2 = pos.substr(2, 2);                                                                //arrotondamento->2 elementi a partire dall'elemento 2, scarto praticamente lo "0."
        outputBuffer.append(pos2);
        pos = std::to_string(m_vecKilobotsPositions[unKilobotID].GetY() + (vArena_size / 2.0));
        pos2 = pos.substr(2, 2);
        outputBuffer.append(pos2);
        /* append 0 for no preferred direction, 1 for left, 2 for right */
        if (GetKilobotLedColor(c_kilobot_entity) == argos::CColor::RED)
        {
            m_vecKilobotStates_ALF[unKilobotID] = COMMITTED_N;
            outputBuffer.append(std::to_string(1)); //kilobot committed for NORTH
        }
        if (GetKilobotLedColor(c_kilobot_entity) == argos::CColor::GREEN)
        {
            m_vecKilobotStates_ALF[unKilobotID] = COMMITTED_S;
            outputBuffer.append(std::to_string(2)); //kilobot committed for SOUTH
        }
        else if (GetKilobotLedColor(c_kilobot_entity) == argos::CColor::BLACK)
        {
            m_vecKilobotStates_ALF[unKilobotID] = UNCOMMITTED;
            outputBuffer.append(std::to_string(0)); //kilobot uncommitted
        }
    }

    /* Task check*/
    if (mode == "CLIENT")
    {
        for (int i = 0; i < multiArea.size(); i++)
        {
            Real fDistance = Distance(m_vecKilobotsPositions[unKilobotID], multiArea[i].Center);
            if ((fDistance < (multiArea[i].Radius * 1.0)) && (multiArea[i].Completed == false))
            {
                multiArea[i].Completed = true;
                std::cout << "Time:" << m_fTimeInSeconds << " exploited area at:" << multiArea[i].Center << (multiArea[i].Color == argos::CColor::RED ? 0 : 1) << std::endl;
                /*Log exploited area as: time - posX - posY - colour(0 for North, 1 for South) */
                m_taskOutput
                    << std::noshowpos
                    << std::setw(8) << std::setprecision(4) << std::setfill('0')
                    << m_fTimeInSeconds << '\t'
                    << std::internal << std::showpos << std::setw(8) << std::setprecision(4) << std::setfill('0') << std::fixed
                    << multiArea[i].Center.GetX() << '\t'
                    << std::internal << std::showpos << std::setw(8) << std::setprecision(4) << std::setfill('0') << std::fixed
                    << multiArea[i].Center.GetY() << '\t'
                    << std::noshowpos << std::setw(1) << std::setprecision(0)
                    << (multiArea[i].Color == argos::CColor::RED ? 0 : 1) << '\t'
                    << std::endl;
                /* Reactivate tasks to keep their number constant */
                std::default_random_engine re;
                re.seed(random_seed);
                if (multiArea[i].Color == argos::CColor::RED)
                {
                    std::uniform_int_distribution<int> distr(0, max_North_area_id);
                    int random_number;
                    do
                    {
                        random_number = distr(re);
                    } while (std::find(activated_North_areas.begin(), activated_North_areas.end(), random_number) != activated_North_areas.end());
                    activated_North_areas.push_back(random_number);
                    multiArea[random_number].Completed = false;
                    activated_North_areas.erase(std::find(activated_North_areas.begin(), activated_North_areas.end(), i));
                    std::sort(activated_North_areas.begin(), activated_North_areas.end());
                }
                else if (multiArea[i].Color == argos::CColor::GREEN)
                {
                    std::uniform_int_distribution<int> distr(max_North_area_id + 1, max_South_area_id);
                    int random_number;
                    do
                    {
                        random_number = distr(re);
                    } while (std::find(activated_South_areas.begin(), activated_South_areas.end(), random_number) != activated_South_areas.end());
                    activated_South_areas.push_back(random_number);
                    multiArea[random_number].Completed = false;
                    activated_South_areas.erase(std::find(activated_South_areas.begin(), activated_South_areas.end(), i));
                    std::sort(activated_South_areas.begin(), activated_South_areas.end());
                }
            }
        }
    }
}

// #ifdef WALL_AVOIDANCE
CVector2 CALFClientServer::VectorRotation2D(Real angle, CVector2 vec)
{
    Real kx = (cos(angle) * vec.GetX()) + (-1.0 * sin(angle) * vec.GetY());
    Real ky = (sin(angle) * vec.GetX()) + (cos(angle) * vec.GetY());
    CVector2 rotated_vector(kx, ky);
    return rotated_vector;
}

std::vector<int> CALFClientServer::Proximity_sensor(CVector2 obstacle_direction, Real kOrientation, int num_sectors)
{
    double sector = M_PI_2 / (num_sectors / 2.0);
    std::vector<int> proximity_values;

    for (int i = 0; i < num_sectors; i++)
    {
        CVector2 sector_dir_a = VectorRotation2D((kOrientation + M_PI_2 - i * sector), left_direction);
        CVector2 sector_dir_b = VectorRotation2D((kOrientation + M_PI_2 - (i + 1) * sector), left_direction);

        if (obstacle_direction.DotProduct(sector_dir_a) >= 0.0 || obstacle_direction.DotProduct(sector_dir_b) >= 0.0)
        {
            proximity_values.push_back(0);
        }
        else
        {
            proximity_values.push_back(1);
        }
    }

    return proximity_values;
}
// #endif

void CALFClientServer::UpdateVirtualSensor(CKilobotEntity &c_kilobot_entity)
{
    m_tALFKilobotMessage tKilobotMessage, tEmptyMessage, tMessage;
    decisionMessage KilobotDecisionMsg, EmptyDecisionMsg, DecisionMsg;
    bool bMessageToSend = false;
    UInt16 unKilobotID = GetKilobotId(c_kilobot_entity);

    /********* WALL AVOIDANCE STUFF *************/
    UInt8 proximity_sensor_dec = 0; //8 bit proximity sensor as decimal

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (mode == "SERVER")
    {
        if (m_fTimeInSeconds - m_vecLastTimeMessaged[unKilobotID] < m_fMinTimeBetweenTwoMsg)
        {
            return;
        }

        else
        {
            float r_on = 0.0, r_off = 0.0, b_on = 0.0, b_off = 0.0;
            for (int i = 0; i < multiArea.size(); i++)
            {
                Real fDistance = Distance(m_vecKilobotsPositions[unKilobotID], multiArea[i].Center);
                if (fDistance <= vision_range)
                {
                    if (multiArea[i].Color == argos::CColor::RED)
                    {
                        if (multiArea[i].Completed == true)
                        {
                            r_off++;
                        }
                        else if (multiArea[i].Completed == false)
                        {
                            r_on++;
                        }
                    }
                    else if (multiArea[i].Color == argos::CColor::GREEN)
                    {
                        if (multiArea[i].Completed == true)
                        {
                            b_off++;
                        }
                        else if (multiArea[i].Completed == false)
                        {
                            b_on++;
                        }
                    }
                }
            }

            if (fabs(m_vecKilobotsPositions[unKilobotID].GetX()) > vDistance_threshold ||
                fabs(m_vecKilobotsPositions[unKilobotID].GetY()) > vDistance_threshold)

            {
                std::vector<int> proximity_vec;
                proximity_bits = 6;

                if (m_vecKilobotsPositions[unKilobotID].GetX() > vDistance_threshold)
                {
                    // std::cerr<<"RIGHT\n";
                    proximity_vec = Proximity_sensor(right_direction, m_vecKilobotsOrientations[unKilobotID].GetValue(), proximity_bits);
                }
                else if (m_vecKilobotsPositions[unKilobotID].GetX() < -1.0 * vDistance_threshold)
                {
                    // std::cerr<<"LEFT\n";
                    proximity_vec = Proximity_sensor(left_direction, m_vecKilobotsOrientations[unKilobotID].GetValue(), proximity_bits);
                }

                if (m_vecKilobotsPositions[unKilobotID].GetY() > vDistance_threshold)
                {
                    // std::cerr<<"UP\n";
                    if (proximity_vec.empty())
                        proximity_vec = Proximity_sensor(up_direction, m_vecKilobotsOrientations[unKilobotID].GetValue(), proximity_bits);
                    else
                    {
                        std::vector<int> prox = Proximity_sensor(up_direction, m_vecKilobotsOrientations[unKilobotID].GetValue(), proximity_bits);
                        std::vector<int> elementwiseOr;
                        elementwiseOr.reserve(prox.size());
                        std::transform(proximity_vec.begin(), proximity_vec.end(), prox.begin(), std::back_inserter(elementwiseOr), std::logical_or<>());

                        proximity_vec = elementwiseOr;
                    }
                }
                else if (m_vecKilobotsPositions[unKilobotID].GetY() < -1.0 * vDistance_threshold)
                {
                    // std::cerr<<"DOWN\n";
                    if (proximity_vec.empty())
                        proximity_vec = Proximity_sensor(down_direction, m_vecKilobotsOrientations[unKilobotID].GetValue(), proximity_bits);
                    else
                    {
                        std::vector<int> prox = Proximity_sensor(up_direction, m_vecKilobotsOrientations[unKilobotID].GetValue(), proximity_bits);
                        std::vector<int> elementwiseOr;
                        elementwiseOr.reserve(prox.size());
                        std::transform(proximity_vec.begin(), proximity_vec.end(), prox.begin(), std::back_inserter(elementwiseOr), std::logical_or<>());

                        proximity_vec = elementwiseOr;
                    }
                }

                proximity_sensor_dec = std::accumulate(proximity_vec.begin(), proximity_vec.end(), 0, [](int x, int y) { return (x << 1) + y; });
                // To turn off the wall avoidance decomment this
                //proximity_sensor_dec = 0;
            }

            // std::cout << r_on << " - " << r_off << " ------ " << b_on << " - " << b_off << std::endl;
            KilobotDecisionMsg.ID = unKilobotID;
            KilobotDecisionMsg.resource_North = (UInt8)std::floor(kResourceNorthBits * (r_on / (r_on + r_off)));
            KilobotDecisionMsg.resource_South = (UInt8)std::floor(kResourceSouthBits * (b_on / (b_on + b_off)));
            KilobotDecisionMsg.wall_avoidance_bits = proximity_sensor_dec;

            // if (unKilobotID == 0)
            // {
            //     std::cout << "N:" << KilobotDecisionMsg.resource_North
            //               << "\tS:" << KilobotDecisionMsg.resource_South
            //               << "\tWA:" << KilobotDecisionMsg.wall_avoidance_bits
            //               << std::endl;
            // }
            m_vecLastTimeMessaged[unKilobotID] = m_fTimeInSeconds;
            bMessageToSend = true;
        }
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////

    if (mode == "CLIENT")
    {

        /*determine kilobot orientation*/
        actual_orientation[unKilobotID] = (int)(m_vecKilobotsOrientations[unKilobotID].GetValue() * 10); //per semplificare i calcoli, quantizzazione di Valerio
        // da 0 a 31 i positivi
        // da 100 a 131 i negativi
        if (actual_orientation[unKilobotID] < 0)
        {
            actual_orientation[unKilobotID] = (-1 * actual_orientation[unKilobotID]) + 100;
        }
        //std::cout<<"act orientation: "<<actual_orientation[unKilobotID]<<std::endl;
        //std::cout<<"command: "<<command[unKilobotID]<<std::endl;

        if (m_fTimeInSeconds - m_vecLastTimeMessaged[unKilobotID] < m_fMinTimeBetweenTwoMsg)
        {
            return;
        }

        else if (fabs(m_vecKilobotsPositions[unKilobotID].GetX()) > vDistance_threshold ||
                 fabs(m_vecKilobotsPositions[unKilobotID].GetY()) > vDistance_threshold)

        {
            std::vector<int> proximity_vec;

            if (m_vecKilobotsPositions[unKilobotID].GetX() > vDistance_threshold)
            {
                // std::cerr<<"RIGHT\n";
                proximity_vec = Proximity_sensor(right_direction, m_vecKilobotsOrientations[unKilobotID].GetValue(), proximity_bits);
            }
            else if (m_vecKilobotsPositions[unKilobotID].GetX() < -1.0 * vDistance_threshold)
            {
                // std::cerr<<"LEFT\n";
                proximity_vec = Proximity_sensor(left_direction, m_vecKilobotsOrientations[unKilobotID].GetValue(), proximity_bits);
            }

            if (m_vecKilobotsPositions[unKilobotID].GetY() > vDistance_threshold)
            {
                // std::cerr<<"UP\n";
                if (proximity_vec.empty())
                    proximity_vec = Proximity_sensor(up_direction, m_vecKilobotsOrientations[unKilobotID].GetValue(), proximity_bits);
                else
                {
                    std::vector<int> prox = Proximity_sensor(up_direction, m_vecKilobotsOrientations[unKilobotID].GetValue(), proximity_bits);
                    std::vector<int> elementwiseOr;
                    elementwiseOr.reserve(prox.size());
                    std::transform(proximity_vec.begin(), proximity_vec.end(), prox.begin(), std::back_inserter(elementwiseOr), std::logical_or<>());

                    proximity_vec = elementwiseOr;
                }
            }
            else if (m_vecKilobotsPositions[unKilobotID].GetY() < -1.0 * vDistance_threshold)
            {
                // std::cerr<<"DOWN\n";
                if (proximity_vec.empty())
                    proximity_vec = Proximity_sensor(down_direction, m_vecKilobotsOrientations[unKilobotID].GetValue(), proximity_bits);
                else
                {
                    std::vector<int> prox = Proximity_sensor(up_direction, m_vecKilobotsOrientations[unKilobotID].GetValue(), proximity_bits);
                    std::vector<int> elementwiseOr;
                    elementwiseOr.reserve(prox.size());
                    std::transform(proximity_vec.begin(), proximity_vec.end(), prox.begin(), std::back_inserter(elementwiseOr), std::logical_or<>());

                    proximity_vec = elementwiseOr;
                }
            }

            proximity_sensor_dec = std::accumulate(proximity_vec.begin(), proximity_vec.end(), 0, [](int x, int y) { return (x << 1) + y; });
            // To turn off the wall avoidance decomment this
            //proximity_sensor_dec = 0;

            /** Print proximity values */
            // std::cerr<<"kID:"<< unKilobotID <<" sensor ";
            // for(int item : proximity_vec)
            // {
            //     std::cerr<< item <<'\t';
            // }
            // std::cerr<<std::endl;

            // std::cout<<"******Prox dec: "<<proximity_sensor_dec<<std::endl;

            tKilobotMessage.m_sID = unKilobotID;
            tKilobotMessage.m_sType = 3;
            tKilobotMessage.m_sData = proximity_sensor_dec;
            bMessageToSend = true;
        }
        // #endif
        else
        {
            /* Compose the message for a kilobot */
            tKilobotMessage.m_sID = unKilobotID;                       //ID of the receiver
            tKilobotMessage.m_sType = (int)command[unKilobotID];       //state
            tKilobotMessage.m_sData = actual_orientation[unKilobotID]; //orientation of the robot
            if ((command[unKilobotID] != 0) && (GetKilobotLedColor(c_kilobot_entity) == argos::CColor::BLACK))
            {
                std::cerr << "sending command: " << (int)command[unKilobotID] << std::endl;
                bMessageToSend = true;
            }
            m_vecLastTimeMessaged[unKilobotID] = m_fTimeInSeconds;
        }
    }

    if (bMessageToSend)
    {
        for (int i = 0; i < 9; ++i)
        {
            m_tMessages[unKilobotID].data[i] = 0;
        }
        tEmptyMessage.m_sID = 1023;
        tEmptyMessage.m_sType = 0;
        tEmptyMessage.m_sData = 0;

        EmptyDecisionMsg.ID = 255;
        EmptyDecisionMsg.resource_North = 0;
        EmptyDecisionMsg.resource_South = 0;
        for (int i = 0; i < 3; ++i)
        {
            /* Packing the message */
            if (mode == "CLIENT")
            {
                if (i == 0)
                {
                    tMessage = tKilobotMessage;
                }
                else
                {
                    tMessage = tEmptyMessage;
                }
                m_tMessages[unKilobotID].data[i * 3] = (tMessage.m_sID >> 2);
                m_tMessages[unKilobotID].data[1 + i * 3] = (tMessage.m_sID << 6);
                m_tMessages[unKilobotID].data[1 + i * 3] = m_tMessages[unKilobotID].data[1 + i * 3] | (tMessage.m_sType << 2);
                m_tMessages[unKilobotID].data[1 + i * 3] = m_tMessages[unKilobotID].data[1 + i * 3] | (tMessage.m_sData >> 8);
                m_tMessages[unKilobotID].data[2 + i * 3] = tMessage.m_sData;
            }
            else if (mode == "SERVER")
            {
                if (i == 0)
                {
                    DecisionMsg = KilobotDecisionMsg;
                }
                else
                {
                    DecisionMsg = EmptyDecisionMsg;
                }
                // m_tMessages[unKilobotID].data[i * 3] = DecisionMsg.ID;
                // m_tMessages[unKilobotID].data[1 + i * 3] = DecisionMsg.resource_North;
                // m_tMessages[unKilobotID].data[2 + i * 3] = DecisionMsg.resource_South;

                m_tMessages[unKilobotID].data[i * 3] = (DecisionMsg.ID << 2);
                m_tMessages[unKilobotID].data[i * 3] = m_tMessages[unKilobotID].data[i * 3] | (DecisionMsg.resource_North >> 4);

                m_tMessages[unKilobotID].data[1 + i * 3] = (DecisionMsg.resource_North << 4);
                m_tMessages[unKilobotID].data[1 + i * 3] = m_tMessages[unKilobotID].data[1 + i * 3] | (DecisionMsg.resource_South >> 2);

                m_tMessages[unKilobotID].data[2 + i * 3] = (DecisionMsg.resource_South << 6);
                m_tMessages[unKilobotID].data[2 + i * 3] = m_tMessages[unKilobotID].data[2 + i * 3] | DecisionMsg.wall_avoidance_bits;
            }
        }
        GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity, &m_tMessages[unKilobotID]);
    }
    else
    {
        GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity, NULL);
    }
}

void CALFClientServer::PostStep()
{
    // std::cout << "Time: " << m_fTimeInSeconds << std::endl;
    internal_counter += 1;
    if (internal_counter % m_unDataAcquisitionFrequency == 0 || internal_counter <= 1)
    {
        KiloLOG();
    }
}

void CALFClientServer::KiloLOG()
{
    // std::cerr << "Logging kiloPosition\n";

    m_kiloOutput
        << std::noshowpos << std::setw(4) << std::setprecision(0) << std::setfill('0')
        << m_fTimeInSeconds << '\t';
    for (size_t kID = 0; kID < m_vecKilobotsPositions.size(); kID++)
    {
        m_kiloOutput
            << std::noshowpos << std::setw(2) << std::setprecision(0) << std::setfill('0')
            << kID << '\t'
            // << std::noshowpos << std::setw(1) << std::setprecision(0)
            // << m_vecKilobotStates_ALF[kID] << '\t' //TODO: this should be the colour, but for now is the state
            << std::internal << std::showpos << std::setw(8) << std::setprecision(4) << std::setfill('0') << std::fixed
            << m_vecKilobotsPositions[kID].GetX() << '\t'
            << std::internal << std::showpos << std::setw(8) << std::setprecision(4) << std::setfill('0') << std::fixed
            << m_vecKilobotsPositions[kID].GetY() << '\t'
            << std::internal << std::showpos << std::setw(6) << std::setprecision(4) << std::setfill('0') << std::fixed
            << m_vecKilobotsOrientations[kID].GetValue() << '\t';
        if (mode == "SERVER")
        {
            m_kiloOutput
                << std::noshowpos << std::setw(1) << std::setprecision(0)
                << m_vecKilobotStates_ALF[kID] << '\t';
        }
    }
    m_kiloOutput << std::endl;
}

CColor CALFClientServer::GetFloorColor(const CVector2 &vec_position_on_plane)
{
    CColor cColor = CColor::WHITE;

    Real fKiloVision = Distance(vec_position_on_plane, m_vecKilobotsPositions[0]);
    if (fKiloVision < vision_range)
    {
        cColor = CColor(0, 0, 125, 0);
    }

    /* Draw areas until they are needed, once that task is completed the corresponding area disappears */
    for (int i = 0; i < multiArea.size(); i++)
    {
        if (multiArea[i].Completed == false)
        {
            Real fDistance = Distance(vec_position_on_plane, multiArea[i].Center);
            if (fDistance < multiArea[i].Radius)
            {
                cColor = multiArea[i].Color;
            }
        }
    }
    return cColor;
}

REGISTER_LOOP_FUNCTIONS(CALFClientServer, "cre_ALF_loop_function")