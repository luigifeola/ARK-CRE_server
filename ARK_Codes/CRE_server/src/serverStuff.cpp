#include "serverStuff.h"

ServerStuff::ServerStuff(QObject *pwgt) : QObject(pwgt), m_nNextBlockSize(0)
{
    tcpServer = new QTcpServer(this);
}

QList<QTcpSocket *> ServerStuff::getClients()
{
    return clients;
}

void ServerStuff::newConnection()
{
    QTcpSocket *clientSocket = tcpServer->nextPendingConnection();

    connect(clientSocket, &QTcpSocket::disconnected, clientSocket, &QTcpSocket::deleteLater);
    connect(clientSocket, &QTcpSocket::readyRead, this, &ServerStuff::readClient);
    connect(clientSocket, &QTcpSocket::disconnected, this, &ServerStuff::gotDisconnection);

    clients << clientSocket;

    sendToClient(clientSocket, "Reply: connection established");
}

void ServerStuff::readClient()
{
    QTcpSocket *clientSocket = (QTcpSocket*)sender();
    quint64 bufferSize = 2048;
    char buffer[bufferSize];
    quint64 dataRead = 0;

    dataRead = clientSocket->read(buffer, bufferSize);
    buffer[dataRead] = 0;

    // qDebug() << "[WEB] Incoming data[" << dataRead << "]: " << buffer;


    emit gotNewMesssage(buffer);

//    if (sendToClient(clientSocket, QString("Reply: received [%1]").arg(buffer)) == -1)
//    {
//        qDebug() << "Some error occured";
//    }
}

void ServerStuff::gotDisconnection()
{
    clients.removeAt(clients.indexOf((QTcpSocket*)sender()));
    emit smbDisconnected();
}

qint64 ServerStuff::sendToClient(QTcpSocket* socket, const QString& str)
{
    //QString str1 = ui->lineEdit_message->text();
    QByteArray ba = str.toLocal8Bit();
    const char *c_str2 = ba.data();
    //client->tcpSocket->write(c_str2);

    return socket->write(c_str2);
}
