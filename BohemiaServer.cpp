// BohemiaServer.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include "Zone/GameServer.cpp"

void handle_client(tcp::socket socket)
{
    try {
        while (true)
        {
            // 1. 读取长度
            uint32_t net_size;
            boost::asio::read(socket, boost::asio::buffer(&net_size, sizeof(net_size)));

            uint32_t size = ntohl(net_size);

            // 2. 按长度读取完整消息
            std::vector<char> buffer(size);
            boost::asio::read(socket, boost::asio::buffer(buffer));

            // 3. protobuf 解析
            MoveRequest req;
            if (!req.ParseFromArray(buffer.data(), buffer.size())) {
                std::cerr << "Failed to parse MoveRequest\n";
                break;
            }

            std::cout << "Player:" << req.player_id()
                << " Pos x:" << req.position_x()
                << " Pos y:" << req.position_y()
                << " Pos z:" << req.position_z()
                << " vx:" << req.velocity_x()
                << " vy:" << req.velocity_y()
                << " vz:" << req.velocity_z() 
                << " horizontal:" << req.horizontal()
                << " vertical:" << req.vertical()
                << std::endl;

            // 4. 构建响应
            MoveBroadcast broadcast;
            broadcast.set_player_id(req.player_id());
            broadcast.set_position_x(req.position_x());
            broadcast.set_position_y(req.position_y());
            broadcast.set_position_z(req.position_z());

            std::string out;
            broadcast.SerializeToString(&out);

            // 5. 发送（带长度）
            uint32_t out_size = htonl(out.size());
            boost::asio::write(socket, boost::asio::buffer(&out_size, 4));
            boost::asio::write(socket, boost::asio::buffer(out));
        }
    }
    catch (const std::exception& e) {
        std::cout << "Client disconnected: " << e.what() << "\n";
    }
}

int main()
{
    GameServer game_server;
    game_server.Initialize();
    game_server.Run1();
    /*try {
        boost::asio::io_context io_context;
        tcp::acceptor acceptor(io_context, tcp::endpoint(tcp::v4(), 8080));

        std::cout << "Server started on port 8080...\n";

        tcp::socket socket(io_context);
        acceptor.accept(socket);
        std::cout << "Client connected.\n";

        handle_client(std::move(socket));
    }
    catch (std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
    }*/
}

// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门使用技巧: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件
