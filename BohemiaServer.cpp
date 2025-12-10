// BohemiaServer.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include "Zone/GameServer.cpp"

int main()
{
    //GameServer game_server;
    //game_server.Initialize();
    //game_server.Run();
    try {
        boost::asio::io_context io_context;
        tcp::acceptor acceptor(io_context, tcp::endpoint(tcp::v4(), 8080));

        std::cout << "Server started on port 8080...\n";

        tcp::socket socket(io_context);
        acceptor.accept(socket);
        std::cout << "Client connected.\n";

        while (true) {
            char data[1024];
            boost::system::error_code ec;

            size_t length = socket.read_some(boost::asio::buffer(data), ec);

            if (ec == boost::asio::error::eof) {
                std::cout << "Client disconnected.\n";
                break;
            }
            else if (ec) {
                std::cerr << "Receive error: " << ec.message() << std::endl;
                break;
            }

            std::string msg(data, length);
            std::cout << "Received: " << msg << "\n";

            // 回显
            std::string reply = "Echo: " + msg;
            boost::asio::write(socket, boost::asio::buffer(reply), ec);
        }
    }
    catch (std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
    }
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
