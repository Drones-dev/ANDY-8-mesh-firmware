#include <assert.h>
#include <stdio.h>
#include "NeighborTable.h"
#include "PacketProtocol.h"
#include "SimulatedRadio.h"
using namespace AndyMesh;
static void sendHello(SimulatedRadioEndpoint& tx,NodeId id,uint32_t now){HelloPayload h{id,id==0?NodeRole::GroundStation:NodeRole::Drone,kProtocolVersion,now};uint8_t payload[kHelloPayloadSize],packet[kMaxPacketSize];size_t n=0;assert(encodeHelloPayload(h,payload,sizeof payload)==ProtocolError::None);PacketHeader ph{kProtocolVersion,PacketType::Hello,id,kBroadcastNodeId,1,now,kHelloPayloadSize};assert(encodePacket(ph,payload,packet,sizeof packet,n)==ProtocolError::None);assert(tx.send(packet,n));}
int main(){SimulatedRadioNetwork net;SimulatedRadioEndpoint e0(net,0),e1(net,1),e2(net,2),e3(net,3);net.addLink({0,1,true,-50,8,10,0});net.addLink({1,2,true,-65,5,10,0});net.addLink({2,3,true,-72,2,10,0});NeighborTable tables[4];for(int i=0;i<4;++i)tables[i].begin(i,5000);SimulatedRadioEndpoint* eps[]={&e0,&e1,&e2,&e3};
 for(int i=0;i<4;++i)sendHello(*eps[i],i,100);net.setNow(110);for(int i=0;i<4;++i){TransportPacket p;while(eps[i]->poll(p)){DecodedPacket d;HelloPayload h;assert(decodePacket(p.data,p.length,d)==ProtocolError::None);assert(decodeHelloPayload(d.payload,d.header.payloadLength,h)==ProtocolError::None);tables[i].observeHello(h,110,p.rssiDbm,p.snrDb);}}
 assert(tables[0].find(1)&&!tables[0].find(2));assert(tables[1].find(0)&&tables[1].find(2));assert(tables[3].find(2)&&!tables[3].find(1));e2.setConnected(false);NodeId lost[8];assert(tables[1].updateTimeouts(5200,lost,8)==2);e2.setConnected(true);net.setNow(5300);sendHello(e2,2,5300);net.setNow(5310);TransportPacket p;assert(e1.poll(p));DecodedPacket d;HelloPayload h;assert(decodePacket(p.data,p.length,d)==ProtocolError::None);assert(decodeHelloPayload(d.payload,d.header.payloadLength,h)==ProtocolError::None);assert(tables[1].observeHello(h,5310,p.rssiDbm,p.snrDb)==NeighborEvent::Recovered);puts("simulation_tests: PASS");}
