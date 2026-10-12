#include "gtest/gtest.h"
#include "flarm_traffic.hh"
#include "flarm_reports.hh"
#include "aircraftjson_utils.hh"
#include "buffer_utils.hh"
#include <array>
#include <cstring>
#include <string>
#include <vector>

namespace {
FlarmSnapshot Snapshot(FlarmSnapshotReceiver& receiver) {
    FlarmSnapshot snapshot;
    uint32_t received_ms;
    receiver.CopySnapshot(snapshot, received_ms);
    return snapshot;
}
uint32_t ReceivedMs(FlarmSnapshotReceiver& receiver) {
    FlarmSnapshot snapshot;
    uint32_t received_ms = 0;
    receiver.CopySnapshot(snapshot, received_ms);
    return received_ms;
}

const uint8_t kPayload[] = {0xfb,0x18,0x0d,0x22,0,0,0xc0,0x03,0x42,0x1c,0x8b,0x57,
                          0xf5,0x77,0x38,0xc2,0x0f,0x29,0xe5,0x61,0x15,0x40,0xa9,0x52};
FlarmRadioPacket Packet(uint32_t sequence = 1) {
    FlarmRadioPacket p;
    p.sequence = sequence;
    p.rssi_dbm = -87;
    memcpy(p.payload, kPayload, 24);
    return p;
}
std::array<uint8_t,54> EncodeFrame() {
    std::array<uint8_t,54> encoded{};
    encoded[0] = 0x65; encoded[1] = 0x96;
    uint8_t frame[26];
    memcpy(frame, kPayload, 24);
    uint16_t crc = famp_crc16(famp_crc16(0xffff, famp_sync, 3), frame, 24);
    frame[24] = crc; frame[25] = crc >> 8;
    for (unsigned bit = 0; bit < 208; ++bit) {
        unsigned pair = frame[bit / 8] & (1 << (7 - bit % 8)) ? 1 : 2;
        encoded[2 + bit / 4] |= pair << (6 - 2 * (bit % 4));
    }
    return encoded;
}
std::vector<std::string> Fields(const char* message) {
    std::vector<std::string> fields;
    std::string s(message);
    size_t start = 0;
    for (size_t pos; (pos = s.find(',', start)) != std::string::npos; start = pos + 1)
        fields.push_back(s.substr(start, pos-start));
    fields.push_back(s.substr(start));
    return fields;
}
}

TEST(Flarm, RadioFrameAndInvertedPolarity) {
    auto frame = EncodeFrame(); uint8_t payload[24];
    ASSERT_TRUE(DecodeFlarmRadioFrame(frame.data(), frame.size(), payload));
    EXPECT_EQ(memcmp(payload, kPayload, 24), 0);
    for (auto& b : frame) b ^= 0xff;
    ASSERT_TRUE(DecodeFlarmRadioFrame(frame.data(), frame.size(), payload));
    EXPECT_EQ(memcmp(payload, kPayload, 24), 0);
}
TEST(Flarm, RejectsCorruptShortAndMalformedManchester) {
    auto frame = EncodeFrame(); uint8_t payload[24];
    EXPECT_FALSE(DecodeFlarmRadioFrame(frame.data(), 53, payload));
    frame[2] ^= 0xc0;
    EXPECT_FALSE(DecodeFlarmRadioFrame(frame.data(), frame.size(), payload));
    frame = EncodeFrame(); frame[2] &= 0x3f;
    EXPECT_FALSE(DecodeFlarmRadioFrame(frame.data(), frame.size(), payload));
    frame = EncodeFrame(); frame[0] ^= 1;
    EXPECT_FALSE(DecodeFlarmRadioFrame(frame.data(), frame.size(), payload));
}
TEST(Flarm, PrivacyMarkedAnnexPacketReachesLocalReportWithNativePrecision) {
    FlarmTraffic traffic;
    ASSERT_TRUE(traffic.Ingest(Packet(), 1000, 1996151268000ULL, true, -273941486, -67004173));
    char message[200];
    traffic.BeginReport(1000);
    ASSERT_GT(traffic.NextReport(message, sizeof(message), 1000), 0);
    auto f = Fields(message);
    ASSERT_EQ(f.size(), 27u);
    EXPECT_EQ(f[0], "#F:1"); EXPECT_EQ(f[1], "F09000"); EXPECT_EQ(f[2], "1");
    EXPECT_EQ(f[3], "3"); EXPECT_EQ(f[4], "8");
    EXPECT_EQ(f[5], "-27.4178008"); EXPECT_EQ(f[6], "-6.7099756");
    EXPECT_EQ(f[7], "11240"); EXPECT_EQ(f[8], "270.0"); EXPECT_EQ(f[9], "176.0");
    EXPECT_EQ(f[10], "56.0"); EXPECT_EQ(f[11], "15.20");
    EXPECT_EQ(f[14], "15.2"); EXPECT_EQ(f[15], "191.00"); EXPECT_EQ(f[16], "1.4");
    EXPECT_EQ(f[20], "1996151268"); EXPECT_EQ(f[21], "0"); EXPECT_EQ(f[24], "-87");
    EXPECT_EQ(f[25], "1");
    size_t crc_offset = std::string(message).rfind(',') + 1;
    EXPECT_EQ(std::stoul(f[26], nullptr, 16), CalculateCRC16(reinterpret_cast<uint8_t*>(message), crc_offset));
    EXPECT_EQ(f[26].substr(f[26].size()-2), "\r\n");
}
TEST(Flarm, MissingReferenceLeavesCoordinatesEmpty) {
    FlarmTraffic traffic;
    ASSERT_TRUE(traffic.Ingest(Packet(), 1000, 1996151268000ULL, false, 0, 0));
    char message[200]; traffic.BeginReport(1000);
    ASSERT_GT(traffic.NextReport(message, sizeof(message), 1000), 0);
    auto f = Fields(message); EXPECT_TRUE(f[5].empty()); EXPECT_TRUE(f[6].empty());
}
TEST(Flarm, ReceptionAgeRepeatedReportsAndExpiry) {
    FlarmTraffic traffic; auto p = Packet(); p.age_ms = 400;
    ASSERT_TRUE(traffic.Ingest(p, 1000, 1996151268400ULL, false, 0, 0));
    char message[200]; traffic.BeginReport(1000);
    ASSERT_GT(traffic.NextReport(message, sizeof(message), 1000), 0);
    EXPECT_EQ(Fields(message)[21], "400");
    EXPECT_EQ(traffic.NextReport(message, sizeof(message), 1000), 0);
    traffic.BeginReport(2000);
    ASSERT_GT(traffic.NextReport(message, sizeof(message), 2000), 0);
    EXPECT_EQ(Fields(message)[21], "1400"); EXPECT_EQ(Fields(message)[25], "0");
    EXPECT_EQ(traffic.NextReport(message, sizeof(message), 2000), 0);
    traffic.BeginReport(5600);
    ASSERT_GT(traffic.NextReport(message, sizeof(message), 5600), 0);
    traffic.BeginReport(60600);
    EXPECT_EQ(traffic.NextReport(message, sizeof(message), 60600), 0);
    p.sequence++; p.age_ms = 5000;
    EXPECT_FALSE(traffic.Ingest(p, 6000, 1996151273000ULL, false, 0, 0));
}
TEST(Flarm, SPIRetryDoesNotDuplicateReceptionsAndSequenceWrapWorks) {
    FlarmTraffic traffic;
    ASSERT_TRUE(traffic.Ingest(Packet(UINT32_MAX-1), 1000, 1996151268000ULL, false, 0, 0));
    ASSERT_TRUE(traffic.Ingest(Packet(UINT32_MAX), 1000, 1996151268000ULL, false, 0, 0));
    EXPECT_FALSE(traffic.Ingest(Packet(UINT32_MAX-1), 1000, 1996151268000ULL, false, 0, 0));
    EXPECT_FALSE(traffic.Ingest(Packet(UINT32_MAX), 1000, 1996151268000ULL, false, 0, 0));
    ASSERT_TRUE(traffic.Ingest(Packet(0), 1000, 1996151268000ULL, false, 0, 0));
    char message[200]; traffic.BeginReport(1000);
    ASSERT_GT(traffic.NextReport(message, sizeof(message), 1000), 0);
    EXPECT_EQ(Fields(message)[25], "3");
}
TEST(Flarm, IdentityNamespacesAndBoundedSerializer) {
    FlarmTarget t; t.data.addr = 0xffffff; t.data.addr_ext = 0xffff; t.data.addr_type = FAMP_ADDR_FLARM_EXT;
    t.id=0xF09000; t.id_type=1;
    char message[200]; ASSERT_GT(WriteCSBeeFlarm(message, sizeof(message), t, 0), 0);
    EXPECT_EQ(Fields(message)[1], "F09000");
    t.data.addr_type = FAMP_ADDR_EPRA; t.data.addr = 0xffffffff; t.data.addr_next = 0x123456;
    ASSERT_GT(WriteCSBeeFlarm(message, sizeof(message), t, 0), 0);
    EXPECT_EQ(Fields(message)[1], "F09000"); EXPECT_EQ(Fields(message).size(),27u);
    char small[5] = {'x','x','x','x','!'};
    EXPECT_EQ(WriteCSBeeFlarm(small, 4, t, 0), -1);
    EXPECT_EQ(small[0], 0); EXPECT_EQ(small[4], '!');
}
TEST(Flarm, ClearRequiresFreshPackets) {
    FlarmTraffic traffic; ASSERT_TRUE(traffic.Ingest(Packet(),1000,1996151268000ULL,false,0,0));
    traffic.Clear(1000); char message[200]; traffic.BeginReport(1000);
    EXPECT_EQ(traffic.NextReport(message,sizeof(message),1000),0);
    ASSERT_TRUE(traffic.Ingest(Packet(),2000,1996151268000ULL,false,0,0));
}

TEST(Flarm, InvalidReconstructedPositionKeepsPrivacyMarkedTrafficVisible) {
    const uint8_t payload[] = {0xfb,0x18,0x0d,0x22,0,0,0xc0,3,0x3c,0x2c,0x93,0xda,
                               0xd4,0xe9,0x47,0xad,0x6f,0x2d,0xc6,0x98,0x48,0x74,0xf1,0x31};
    auto p = Packet(); memcpy(p.payload, payload, sizeof(payload));
    FlarmTraffic traffic;
    ASSERT_TRUE(traffic.Ingest(p, 1000, 1996151268000ULL, true, 900000000, 1800000000));
    char message[200]; traffic.BeginReport(1000);
    ASSERT_GT(traffic.NextReport(message, sizeof(message), 1000), 0);
    auto f = Fields(message);
    EXPECT_EQ(f[1], "F09000"); EXPECT_EQ(f[3], "3");
    EXPECT_TRUE(f[5].empty()); EXPECT_TRUE(f[6].empty());
}
TEST(Flarm, CoordinateResolutionRejectsOverflowBeforeNarrowing) {
    famp_data_t data{}; data.lat_grid = 338168; data.lon_grid = 849746;
    famp_position_t position{123,456};
    EXPECT_EQ(famp_resolve_position(&data, 890000000, 1800000000, &position), FAMP_ERR_POSITION);
    EXPECT_EQ(position.lat, 123); EXPECT_EQ(position.lon, 456);
    EXPECT_EQ(famp_resolve_position(&data, INT32_MIN, 0, &position), FAMP_ERR_POSITION);
    EXPECT_EQ(famp_resolve_position(&data, 0, INT32_MAX, &position), FAMP_ERR_POSITION);
}

namespace {
// Reception ending at cycle phase p of a cycle whose phase 0 is utc_offset ms past boot.
uint32_t RxEnd(uint32_t utc_offset, uint32_t cycle, uint16_t p) {
    return utc_offset + cycle * 1000 + p + FlarmPhaseEstimator::kPacketMs;
}
}
TEST(FlarmPhase, StaysOnChannel1WithoutSpread) {
    FlarmPhaseEstimator e;
    EXPECT_EQ(e.Channel(0), 1);
    for (uint32_t c = 0; c < 10; ++c) e.Record(RxEnd(777, c, 460 + c * 25), 1);
    EXPECT_EQ(e.Channel(RxEnd(777, 10, 0)), 1);
    EXPECT_EQ(e.Channel(RxEnd(777, 10, 600)), 1);
}
TEST(FlarmPhase, HopsOnceSpreadCoversFrameAndHealsOnSilence) {
    FlarmPhaseEstimator e;
    uint32_t off = 777;
    for (uint32_t c = 0; c < 6; ++c) e.Record(RxEnd(off, c, 455 + c * 60), 1);  // 300 ms spread, +35 ms bias.
    for (uint16_t p : {0, 100, 280}) EXPECT_EQ(e.Channel(RxEnd(off, 7, p)), 2) << p;
    for (uint16_t p : {340, 500, 780}) EXPECT_EQ(e.Channel(RxEnd(off, 7, p)), 1) << p;
    e.Record(RxEnd(off, 8, 900), 2);  // Frame 2 reception maps into Frame 1.
    EXPECT_EQ(e.Channel(RxEnd(off, 9, 100)), 2);
    EXPECT_EQ(e.Channel(RxEnd(off, 9, 100) + FlarmPhaseEstimator::kExpiryMs), 1);
}
TEST(FlarmPhase, ContradictingTimestampsFallBack) {
    FlarmPhaseEstimator e;
    for (uint16_t p = 0; p < 1000; p += 50) e.Record(1000 + p, 1);
    EXPECT_EQ(e.Channel(3000), 1);
}
TEST(FlarmPhase, WrapAroundSecondBoundary) {
    FlarmPhaseEstimator e;
    uint32_t off = 400;  // Frame 1 straddles the ms%1000 wrap.
    for (uint32_t c = 0; c < 6; ++c) e.Record(RxEnd(off, c, 455 + c * 60), 1);
    EXPECT_EQ(e.Channel(RxEnd(off, 7, 500)), 1);
    EXPECT_EQ(e.Channel(RxEnd(off, 7, 900)), 2);
}

namespace {
// FTD-116 sections 4.1/4.2: synthetic receive-only fixtures. Never transmitted.
struct FampFixture {
    uint32_t address = 0x455313db, utc = 1600000000;
    uint8_t type = FAMP_ADDR_EPRA, random = 0, privacy = 0, aircraft = 1, movement = 2, urgency = 0;
    uint16_t extension = 0;
    uint8_t hacc = 8, vacc = 4, nic = 11;
    FlarmRadioPacket Packet(uint32_t sequence) const {
        FlarmRadioPacket p{}; p.sequence = sequence; p.rssi_dbm = -70;
        auto put = [](uint8_t* b, unsigned pos, unsigned bits, uint32_t v) {
            for (unsigned i = 0; i < bits; ++i) b[(pos+i)/8] |= ((v>>i)&1) << ((pos+i)%8);
        };
        put(p.payload, 0, 24, address); put(p.payload, 24, 4, 2);
        put(p.payload, 28, 2, type); put(p.payload, 30, 2, urgency);
        put(p.payload, 32, 16, type == FAMP_ADDR_EPRA ? random | ((address>>24)<<8) : extension);
        put(p.payload, 54, 2, privacy);
        uint8_t data[17] = {0x33};
        put(data, 8, 6, (utc%16)*4); put(data, 14, 5, aircraft);
        put(data, 19, 13, 2000); // 1000 m HAE.
        put(data, 32, 20, 100); put(data, 52, 20, 200);
        put(data, 81, 10, 100); // 10 m/s.
        put(data, 100, 10, 180); // 90 degrees.
        put(data, 110, 2, movement); put(data, 112, 6, hacc);
        put(data, 118, 5, vacc); put(data, 132, 4, nic);
        p.payload[7] = data[0];
        uint8_t nonce[16] = {}; memcpy(nonce, p.payload, 8);
        put(nonce, 64, 32, utc/16); put(nonce, 96, 32, 0x956f6c77);
        auto mix = [](uint32_t z, uint32_t y, uint32_t sum) {
            return (((z>>5)^(y<<2))+((y>>3)^(z<<4))) ^ (sum^y);
        };
        uint32_t sum = 0;
        for (int r=0; r<2; ++r) {
            sum += 0x9e3779b9; uint32_t z = nonce[15];
            for (int i=0; i<16; ++i) { nonce[i] += mix(z,nonce[(i+1)%16],sum); z=nonce[i]; }
        }
        uint32_t words[4] = {}, key[] = {0xa5f9b21c,0xab3f9d12,0xc6f34e34,0xd72fa378};
        for (int i=0; i<16; ++i) words[i/4] |= uint32_t(data[i+1]^nonce[i])<<((i%4)*8);
        sum=0;
        for (int r=0; r<6; ++r) {
            sum += 0x9e3779b9; unsigned e=(sum>>2)&3; uint32_t z=words[3];
            for (unsigned i=0; i<4; ++i) {
                uint32_t y=words[(i+1)%4];
                words[i] += (((z>>5)^(y<<2))+((y>>3)^(z<<4))) ^ ((sum^y)+(key[(i&3)^e]^z));
                z=words[i];
            }
        }
        for (int i=0; i<16; ++i) p.payload[8+i]=words[i/4]>>((i%4)*8);
        return p;
    }
};
std::vector<std::vector<std::string>> Reports(FlarmTraffic& traffic, uint32_t now) {
    std::vector<std::vector<std::string>> out; char line[200]; traffic.BeginReport(now);
    for (int n; (n=traffic.NextReport(line,sizeof(line),now))>0;) out.push_back(Fields(line));
    return out;
}
}
TEST(Flarm, FollowsAdvertisedRotationAfterSixSecondGap) {
    FlarmTraffic traffic; FampFixture f;
    ASSERT_TRUE(traffic.Ingest(f.Packet(1),1000,uint64_t(f.utc)*1000,true,0,0));
    auto before=Reports(traffic,1000); ASSERT_EQ(before.size(),1u);
    EXPECT_EQ(before[0][5],"0.0005200"); EXPECT_EQ(before[0][6],"0.0010400");
    f.address=0xbca14d8e; f.utc+=6;
    ASSERT_TRUE(traffic.Ingest(f.Packet(2),7000,uint64_t(f.utc)*1000,true,0,0));
    auto after=Reports(traffic,7000); ASSERT_EQ(after.size(),1u);
    EXPECT_EQ(after[0][1],before[0][1]);
}

TEST(Flarm, ExhaustedAddressesSuppressEmissionUntilHoldExpires) {
    FlarmTraffic traffic; FampFixture f; f.type=FAMP_ADDR_FLARM;
    for (unsigned i=0; i<1024; ++i) {
        f.address=i+1;
        ASSERT_TRUE(traffic.Ingest(f.Packet(i+1),1000,uint64_t(f.utc)*1000,true,0,0));
    }
    f.address=2000;
    ASSERT_TRUE(traffic.Ingest(f.Packet(1025),1001,uint64_t(f.utc)*1000,true,0,0));
    auto reports=Reports(traffic,1001);
    ASSERT_EQ(reports.size(),63u);
    for (auto& r : reports) EXPECT_LE(std::stoul(r[1],nullptr,16),0xF093FFu);
    f.utc+=60;
    ASSERT_TRUE(traffic.Ingest(f.Packet(1026),61001,uint64_t(f.utc)*1000,true,0,0));
    reports=Reports(traffic,61001); ASSERT_EQ(reports.size(),1u);
    EXPECT_GE(std::stoul(reports[0][1],nullptr,16),0xF09000u);
    EXPECT_LE(std::stoul(reports[0][1],nullptr,16),0xF093FFu);
}

namespace {
std::vector<uint8_t> Unframe(const uint8_t* bytes, size_t length) {
    std::vector<uint8_t> result;
    for (size_t i=1; i+1<length; ++i) {
        if (bytes[i]==0x7d && i+2<length) result.push_back(bytes[++i]^0x20);
        else result.push_back(bytes[i]);
    }
    return result;
}
}
TEST(Flarm, ReceivedFixUsesResolvedIdentityAndGDL90Mappings) {
    FlarmTraffic traffic; FampFixture f; f.urgency=2;
    ASSERT_TRUE(traffic.Ingest(f.Packet(1),1000,uint64_t(f.utc)*1000,true,0,0));
    FlarmSnapshot snapshot; traffic.MakeSnapshot(snapshot,1000);
    FlarmReports output; output.Update(snapshot,1000);
    AircraftDictionary adsb; uint8_t bytes[64];
    unsigned n=output.WriteGDL90(0,bytes,sizeof(bytes),1000,adsb);
    ASSERT_GT(n,0u); EXPECT_EQ(bytes[0],0x7e); EXPECT_EQ(bytes[n-1],0x7e);
    auto b=Unframe(bytes,n); ASSERT_EQ(b.size(),30u);
    EXPECT_EQ(b[0],20); EXPECT_EQ(b[1],1);
    EXPECT_EQ(b[2],0xf0); EXPECT_EQ(b[3],0x90); EXPECT_EQ(b[4],0);
    EXPECT_EQ(b[11],0xff); EXPECT_EQ(b[12],0xf9); // unavailable altitude, updated/airborne/true track
    EXPECT_EQ(b[13],0x8b); EXPECT_EQ(b[17],64); // NIC8, NACp11, eastbound
    EXPECT_EQ(b[18],9); EXPECT_EQ(b[27],0x10); // glider, general emergency
    EXPECT_EQ(std::string(reinterpret_cast<char*>(&b[19]),8),"GLDR9000");
    auto local=Reports(traffic,1000); ASSERT_EQ(local.size(),1u); EXPECT_EQ(local[0][1],"F09000");
    EXPECT_EQ(output.WriteGDL90(0,bytes,sizeof(bytes),1100,adsb),0u);
    traffic.MakeSnapshot(snapshot,2000); output.Update(snapshot,2000);
    EXPECT_EQ(output.WriteGDL90(0,bytes,sizeof(bytes),2000,adsb),0u);
}

TEST(Flarm, AircraftTypesTranslateWithoutInventingWeightClasses) {
    const uint8_t expected[] = {0,9,0,7,11,0,12,12,0,0,7,10,10,14,0,19,0,14,14,14};
    for (unsigned type=0; type<32; ++type) {
        FlarmTraffic traffic; FampFixture f; f.aircraft=type;
        ASSERT_TRUE(traffic.Ingest(f.Packet(1),1000,uint64_t(f.utc)*1000,true,0,0));
        FlarmSnapshot snapshot; traffic.MakeSnapshot(snapshot,1000);
        FlarmReports output; output.Update(snapshot,1000); AircraftDictionary adsb; uint8_t bytes[64];
        unsigned n=output.WriteGDL90(0,bytes,sizeof(bytes),1000,adsb);
        ASSERT_GT(n,0u); auto b=Unframe(bytes,n);
        EXPECT_EQ(b[18],type<20 ? expected[type] : 0) << type;
    }
}

TEST(FlarmSnapshot, PublishesCompleteSnapshotsAndAccountsForTransferAge) {
    FlarmTraffic traffic; FampFixture f;
    ASSERT_TRUE(traffic.Ingest(f.Packet(1),1000,uint64_t(f.utc)*1000,true,0,0));
    FlarmSnapshot snapshot; traffic.MakeSnapshot(snapshot,1300);
    FlarmSnapshotReceiver receiver;
    const auto* bytes=reinterpret_cast<const uint8_t*>(&snapshot);
    EXPECT_TRUE(receiver.Write(bytes,512,0,5000));
    EXPECT_EQ(Snapshot(receiver).count,0);
    EXPECT_FALSE(receiver.Write(bytes+1024,128,1024,5050));
    EXPECT_EQ(Snapshot(receiver).count,0);
    ASSERT_TRUE(receiver.Write(bytes,512,0,5100));
    for (size_t off=512; off<sizeof(snapshot); off+=512)
        ASSERT_TRUE(receiver.Write(bytes+off,std::min(size_t(512),sizeof(snapshot)-off),off,5200));
    ASSERT_EQ(Snapshot(receiver).count,1);
    EXPECT_EQ(Snapshot(receiver).targets[0].rx_age_ms,300u);
    EXPECT_EQ(ReceivedMs(receiver),5100u);
    FlarmSnapshot empty; empty.generation=snapshot.generation+1;
    ASSERT_TRUE(receiver.Write(reinterpret_cast<uint8_t*>(&empty),sizeof(empty),0,6000));
    EXPECT_EQ(Snapshot(receiver).count,0);
}

TEST(FlarmSnapshot, RejectsMalformedRecordsWithoutReplacingPublishedState) {
    FlarmTraffic traffic; FampFixture f;
    ASSERT_TRUE(traffic.Ingest(f.Packet(1),1000,uint64_t(f.utc)*1000,true,0,0));
    FlarmSnapshot good; traffic.MakeSnapshot(good,1000); FlarmSnapshotReceiver receiver;
    ASSERT_TRUE(receiver.Write(reinterpret_cast<uint8_t*>(&good),sizeof(good),0,100));
    FlarmReports consumer; ASSERT_TRUE(consumer.Update(receiver)); ASSERT_EQ(consumer.Count(),1u);
    for (unsigned fault=0; fault<9; ++fault) {
        auto bad=good; bad.generation++;
        switch (fault) {
            case 0: bad.count=65; break;
            case 1: bad.version=2; break;
            case 2: bad.targets[0].lat=INT32_MAX; break;
            case 3: bad.targets[0].id=0x12345678; break;
            case 4: bad.targets[0].flags=128; break;
            case 5: bad.targets[0].position_valid=2; break;
            case 6: bad.targets[0].track=65535; break;
            case 7: bad.targets[0].reserved[0]=1; break;
            case 8: bad.count=2; bad.targets[1]=bad.targets[0]; break;
        }
        EXPECT_FALSE(receiver.Write(reinterpret_cast<uint8_t*>(&bad),sizeof(bad),0,200)) << fault;
        EXPECT_FALSE(consumer.Update(receiver)) << fault;
        EXPECT_EQ(consumer.Count(),1u) << fault;
    }
    auto old=good; old.generation++; old.targets[0].rx_age_ms=59999; old.targets[0].position_age_ms=66000;
    EXPECT_TRUE(receiver.Write(reinterpret_cast<uint8_t*>(&old),sizeof(old),0,250));
    old.generation++; old.targets[0].position_age_ms=59998;
    EXPECT_FALSE(receiver.Write(reinterpret_cast<uint8_t*>(&old),sizeof(old),0,260));
    EXPECT_FALSE(receiver.Write(nullptr,1,0,300));
    EXPECT_FALSE(receiver.Write(reinterpret_cast<uint8_t*>(&good),SIZE_MAX,1,300));
}

TEST(Flarm, DelayedSnapshotsDoNotTurnRetainedCoordinatesIntoUpdates) {
    FlarmTraffic traffic; FampFixture f;
    ASSERT_TRUE(traffic.Ingest(f.Packet(1),1000,uint64_t(f.utc)*1000,true,0,0));
    FlarmSnapshot snapshot; traffic.MakeSnapshot(snapshot,1000);
    FlarmReports output; output.Update(snapshot,5000); AircraftDictionary adsb; uint8_t bytes[64];
    EXPECT_EQ(output.WriteGDL90(0,bytes,sizeof(bytes),11001,adsb),0u);
    traffic.MakeSnapshot(snapshot,21000); output.Update(snapshot,25000);
    EXPECT_EQ(output.WriteGDL90(0,bytes,sizeof(bytes),25000,adsb),0u);
    f.utc+=21;
    ASSERT_TRUE(traffic.Ingest(f.Packet(2),22000,uint64_t(f.utc)*1000,true,0,0));
    traffic.MakeSnapshot(snapshot,22000); output.Update(snapshot,26000);
    EXPECT_GT(output.WriteGDL90(0,bytes,sizeof(bytes),26000,adsb),0u);
}

TEST(Flarm, ExplicitICAOUsesFreshestSourceWithADSBWinningFreshTies) {
    FlarmTraffic traffic; FampFixture f; f.type=FAMP_ADDR_ICAO; f.address=0xADF001;
    ASSERT_TRUE(traffic.Ingest(f.Packet(1),20000,uint64_t(f.utc)*1000,true,0,0));
    FlarmSnapshot snapshot; traffic.MakeSnapshot(snapshot,20000);
    FlarmReports output; output.Update(snapshot,20000); AircraftDictionary adsb;
    ModeSAircraft ac(f.address); ac.WriteBitFlag(ModeSAircraft::kBitFlagPositionValid,true);
    ac.last_position_update_ms=20000; ASSERT_NE(adsb.InsertAircraft(ac),nullptr);
    auto uid=Aircraft::ICAOToUID(f.address,Aircraft::kAircraftTypeModeS);
    uint8_t bytes[64];
    EXPECT_EQ(output.WriteGDL90(0,bytes,sizeof(bytes),20000,adsb),0u);
    EXPECT_FALSE(output.SuppressesAircraft(uid,adsb,20000));
    adsb.GetAircraftPtr<ModeSAircraft>(f.address)->last_position_update_ms=1000;
    EXPECT_GT(output.WriteGDL90(0,bytes,sizeof(bytes),20000,adsb),0u);
    EXPECT_TRUE(output.SuppressesAircraft(uid,adsb,20000));
    EXPECT_FALSE(output.SuppressesAircraft(uid,adsb,81000));
    EXPECT_EQ(output.WriteGDL90(0,bytes,sizeof(bytes),20000,adsb),0u);
    EXPECT_TRUE(output.SuppressesAircraft(uid,adsb,20000));  // Sent fixes still outrank stale ADS-B.
}

TEST(Flarm, StaleFlarmFixNeitherSuppressesNorReports) {
    FlarmTraffic traffic; FampFixture f; f.type=FAMP_ADDR_ICAO; f.address=0xADF002;
    ASSERT_TRUE(traffic.Ingest(f.Packet(1),20000,uint64_t(f.utc)*1000,true,0,0));
    FlarmSnapshot snapshot; traffic.MakeSnapshot(snapshot,20000);
    FlarmReports output; output.Update(snapshot,20000); AircraftDictionary adsb;
    ModeSAircraft ac(f.address); ac.WriteBitFlag(ModeSAircraft::kBitFlagPositionValid,true);
    ac.last_position_update_ms=1000; ASSERT_NE(adsb.InsertAircraft(ac),nullptr);
    auto uid=Aircraft::ICAOToUID(f.address,Aircraft::kAircraftTypeModeS);
    uint8_t bytes[64];
    EXPECT_TRUE(output.SuppressesAircraft(uid,adsb,26000));
    EXPECT_FALSE(output.SuppressesAircraft(uid,adsb,26001));
    EXPECT_EQ(output.WriteGDL90(0,bytes,sizeof(bytes),26001,adsb),0u);
}

TEST(Flarm, SyntheticIDsAvoidOtherExportedTypeOneAddresses) {
    AircraftDictionary adsb; RemoteIDAircraft drone(0xF09000);
    ASSERT_NE(adsb.InsertAircraft(drone),nullptr);
    FlarmTraffic traffic; traffic.SetAddressReservations(adsb,1000); FampFixture f;
    ASSERT_TRUE(traffic.Ingest(f.Packet(1),1000,uint64_t(f.utc)*1000,true,0,0));
    auto local=Reports(traffic,1000); ASSERT_EQ(local.size(),1u); EXPECT_EQ(local[0][1],"F09001");
    FlarmSnapshot snapshot; traffic.MakeSnapshot(snapshot,1000);
    FlarmReports output; output.Update(snapshot,1000);
    RemoteIDAircraft later(0xF09001); ASSERT_NE(adsb.InsertAircraft(later),nullptr);
    uint8_t bytes[64]; EXPECT_EQ(output.WriteGDL90(0,bytes,sizeof(bytes),1000,adsb),0u);
    traffic.SetAddressReservations(adsb,1001);
    local=Reports(traffic,1001); ASSERT_EQ(local.size(),1u); EXPECT_EQ(local[0][1],"F09002");
}

TEST(FlarmSnapshot, RetriesKeepOriginalAgeAndTimedOutTransfersStayPrivate) {
    FlarmTraffic traffic; FampFixture f;
    ASSERT_TRUE(traffic.Ingest(f.Packet(1),1000,uint64_t(f.utc)*1000,true,0,0));
    FlarmSnapshot snapshot; traffic.MakeSnapshot(snapshot,1000);
    auto* bytes=reinterpret_cast<uint8_t*>(&snapshot); FlarmSnapshotReceiver receiver;
    ASSERT_TRUE(receiver.Write(bytes,512,0,100));
    ASSERT_TRUE(receiver.Write(bytes,512,0,110));
    ASSERT_TRUE(receiver.Write(bytes+512,512,512,120));
    ASSERT_TRUE(receiver.Write(bytes+512,512,512,130));
    ASSERT_TRUE(receiver.Write(bytes+1024,sizeof(snapshot)-1024,1024,140));
    EXPECT_EQ(ReceivedMs(receiver),100u);
    ASSERT_TRUE(receiver.Write(bytes+1024,sizeof(snapshot)-1024,1024,150));
    EXPECT_EQ(ReceivedMs(receiver),100u);
    FlarmReports consumer; ASSERT_TRUE(consumer.Update(receiver));
    snapshot.generation++;
    ASSERT_TRUE(receiver.Write(bytes,512,0,1000));
    EXPECT_FALSE(receiver.Write(bytes+512,sizeof(snapshot)-512,512,5000));
    EXPECT_FALSE(consumer.Update(receiver)); EXPECT_EQ(consumer.Count(),1u);
}

TEST(Flarm, RadioResetHoldsReleasedIDsAndRestartedSequenceIsAccepted) {
    FlarmTraffic traffic; FampFixture f; f.type=FAMP_ADDR_FLARM;
    for (unsigned i=0; i<1024; ++i) {
        f.address=i+1; ASSERT_TRUE(traffic.Ingest(f.Packet(i+1),1000,uint64_t(f.utc)*1000,true,0,0));
    }
    traffic.Clear(1001); f.address=2000;
    ASSERT_TRUE(traffic.Ingest(f.Packet(1),1002,uint64_t(f.utc)*1000,true,0,0));
    EXPECT_TRUE(Reports(traffic,1002).empty());
}

TEST(Flarm, OwnshipICAOIsNotReportedAsTraffic) {
    FlarmTraffic traffic; FampFixture f; f.type=FAMP_ADDR_ICAO; f.address=0xADF001;
    ASSERT_TRUE(traffic.Ingest(f.Packet(1),1000,uint64_t(f.utc)*1000,true,0,0));
    FlarmSnapshot snapshot; traffic.MakeSnapshot(snapshot,1000);
    FlarmReports output; output.Update(snapshot,1000); AircraftDictionary adsb; uint8_t bytes[64];
    EXPECT_EQ(output.WriteGDL90(0,bytes,sizeof(bytes),1000,adsb,f.address),0u);
    EXPECT_GT(output.WriteGDL90(0,bytes,sizeof(bytes),1000,adsb,0xADF002),0u);
}

TEST(Flarm, AmbiguousRotationAllocatesOnceAndCurrentAddressWins) {
    FlarmTraffic traffic; FampFixture f;
    ASSERT_TRUE(traffic.Ingest(f.Packet(1),1000,uint64_t(f.utc)*1000,true,0,0));
    f.address=0x455313da; f.random=1;
    ASSERT_TRUE(traffic.Ingest(f.Packet(2),1000,uint64_t(f.utc)*1000,true,0,0));
    f.address=0xbca14d8e; f.random=0; f.utc+=2;
    ASSERT_TRUE(traffic.Ingest(f.Packet(3),3000,uint64_t(f.utc)*1000,true,0,0));
    auto first=Reports(traffic,3000); ASSERT_EQ(first.size(),3u);
    ASSERT_TRUE(traffic.Ingest(f.Packet(4),3100,uint64_t(f.utc)*1000,true,0,0));
    auto second=Reports(traffic,3100); ASSERT_EQ(second.size(),3u);
    for (size_t i=0; i<first.size(); ++i) EXPECT_EQ(first[i][1],second[i][1]);
    FlarmSnapshot snapshot; traffic.MakeSnapshot(snapshot,3100);
    FlarmReports output; output.Update(snapshot,3100); AircraftDictionary adsb; uint8_t bytes[64];
    for (size_t i=0; i<output.Count(); ++i) EXPECT_GT(output.WriteGDL90(i,bytes,sizeof(bytes),3100,adsb),0u);
}

TEST(Flarm, PrivacyAndMissingPositionSuppressGDL90ButRemainInLocalReport) {
    for (unsigned flags=0; flags<4; ++flags) {
        FlarmTraffic traffic; FampFixture f; f.privacy=flags;
        ASSERT_TRUE(traffic.Ingest(f.Packet(1),1000,uint64_t(f.utc)*1000,true,0,0));
        FlarmSnapshot snapshot; traffic.MakeSnapshot(snapshot,1000);
        FlarmReports output; output.Update(snapshot,1000); AircraftDictionary adsb; uint8_t bytes[65];
        memset(bytes,0xa5,sizeof(bytes));
        EXPECT_EQ(output.WriteGDL90(0,bytes,63,1000,adsb),0u);
        EXPECT_EQ(bytes[63],0xa5);
        unsigned n=output.WriteGDL90(0,bytes,64,1000,adsb);
        EXPECT_EQ(n>0,flags==0); EXPECT_EQ(bytes[64],0xa5);
        auto local=Reports(traffic,1000); ASSERT_EQ(local.size(),1u);
        EXPECT_EQ(local[0][3],std::to_string(flags));
        f.utc++;
        ASSERT_TRUE(traffic.Ingest(f.Packet(2),2000,uint64_t(f.utc)*1000,false,0,0));
        traffic.MakeSnapshot(snapshot,2000); output.Update(snapshot,2000);
        EXPECT_EQ(output.WriteGDL90(0,bytes,64,2000,adsb),0u);
        local=Reports(traffic,2000); ASSERT_EQ(local.size(),1u); EXPECT_TRUE(local[0][5].empty());
    }
}

TEST(Flarm, NewStationaryFixReportsOnceAcrossMillisecondWrap) {
    FlarmTraffic traffic; FampFixture f;
    ASSERT_TRUE(traffic.Ingest(f.Packet(1),UINT32_MAX-499,uint64_t(f.utc)*1000,true,0,0));
    FlarmSnapshot snapshot; traffic.MakeSnapshot(snapshot,UINT32_MAX-499);
    FlarmReports output; output.Update(snapshot,UINT32_MAX-499); AircraftDictionary adsb; uint8_t bytes[64];
    ASSERT_GT(output.WriteGDL90(0,bytes,sizeof(bytes),100,adsb),0u);
    EXPECT_EQ(output.WriteGDL90(0,bytes,sizeof(bytes),200,adsb),0u);
    f.utc++;
    ASSERT_TRUE(traffic.Ingest(f.Packet(2),500,uint64_t(f.utc)*1000,true,0,0));
    traffic.MakeSnapshot(snapshot,500); output.Update(snapshot,500);
    EXPECT_GT(output.WriteGDL90(0,bytes,sizeof(bytes),500,adsb),0u);
}

TEST(Flarm, CoprocessorRestartPreservesIdentityAcrossAdvertisedRotation) {
    FlarmTraffic traffic; FampFixture f;
    ASSERT_TRUE(traffic.Ingest(f.Packet(100),1000,uint64_t(f.utc)*1000,true,0,0));
    auto before=Reports(traffic,1000); ASSERT_EQ(before.size(),1u);
    traffic.ResetSequence();
    f.address=0xbca14d8e; f.utc+=2;
    ASSERT_TRUE(traffic.Ingest(f.Packet(1),3000,uint64_t(f.utc)*1000,true,0,0));
    auto after=Reports(traffic,3000); ASSERT_EQ(after.size(),1u); EXPECT_EQ(before[0][1],after[0][1]);
    FlarmSnapshot snapshot; traffic.MakeSnapshot(snapshot,3000);
    FlarmReports output; output.Update(snapshot,3000); AircraftDictionary adsb; uint8_t bytes[64];
    EXPECT_GT(output.WriteGDL90(0,bytes,sizeof(bytes),3000,adsb),0u);
}

TEST(Flarm, EmissionBudgetIncludesBothReportingDelays) {
    for (uint32_t age : {1200u, 2000u}) {
        FlarmTraffic traffic; FampFixture f;
        ASSERT_TRUE(traffic.Ingest(f.Packet(1),10000,uint64_t(f.utc)*1000+age,true,0,0));
        FlarmSnapshot snapshot; traffic.MakeSnapshot(snapshot,10990);
        FlarmReports output; output.Update(snapshot,10990);
        AircraftDictionary adsb; uint8_t bytes[64];
        EXPECT_EQ(Reports(traffic,11980).size(),1u);
        EXPECT_GT(output.WriteGDL90(0,bytes,sizeof(bytes),11980,adsb),0u) << age;
        EXPECT_EQ(output.WriteGDL90(0,bytes,sizeof(bytes),11980,adsb),0u) << age;
        FlarmReports late; late.Update(snapshot,10990);
        EXPECT_GT(late.WriteGDL90(0,bytes,sizeof(bytes),16000-age,adsb),0u) << age;
        EXPECT_EQ(late.WriteGDL90(0,bytes,sizeof(bytes),16000-age,adsb),0u) << age;
        FlarmReports delayed; delayed.Update(snapshot,10990);
        EXPECT_EQ(delayed.WriteGDL90(0,bytes,sizeof(bytes),16001-age,adsb),0u) << age;
    }
}

TEST(Flarm, ContinuousReceptionSurvivesFreeRunningReportingPhases) {
    FlarmTraffic traffic; FampFixture f; FlarmReports output; AircraftDictionary adsb;
    uint32_t next_snapshot=2500, next_report=3001, sequence=0, reports=0, emitted=0;
    const uint32_t epoch=f.utc;
    for (uint32_t now=2000; now<53000; ++now) {
        if (now%1000==0) {
            f.utc=epoch+(now-2000)/1000;
            ASSERT_TRUE(traffic.Ingest(f.Packet(++sequence),now,uint64_t(f.utc)*1000+2000,true,0,0));
        }
        if (now==next_snapshot) {
            FlarmSnapshot snapshot; traffic.MakeSnapshot(snapshot,now); output.Update(snapshot,now);
            next_snapshot+=1001;
        }
        if (now==next_report) {
            uint8_t bytes[64]; ++reports;
            EXPECT_EQ(Reports(traffic,now).size(),1u);
            if (output.WriteGDL90(0,bytes,sizeof(bytes),now,adsb)) ++emitted;
            next_report+=1010;
        }
    }
    EXPECT_EQ(reports,50u);
    EXPECT_EQ(emitted,reports);
}

TEST(Flarm, MapRepeatsRetainedStateWithOriginalAgesAndGeometricAltitude) {
    FlarmTraffic traffic; FampFixture f;
    ASSERT_TRUE(traffic.Ingest(f.Packet(1),1000,uint64_t(f.utc)*1000,true,0,0));
    FlarmSnapshot snapshot; traffic.MakeSnapshot(snapshot,1500);
    FlarmReports output; output.Update(snapshot,5000); AircraftDictionary adsb; char json[512];
    ASSERT_GT(output.WriteJSON(0,json,sizeof(json),20500,adsb),0);
    std::string text(json);
    EXPECT_NE(text.find("\"hex\":\"~f09000\""),std::string::npos);
    EXPECT_NE(text.find("\"type\":\"flarm\""),std::string::npos);
    EXPECT_NE(text.find("\"alt_geom\":3280.8"),std::string::npos);
    EXPECT_EQ(text.find("alt_baro"),std::string::npos);
    EXPECT_NE(text.find("\"seen_pos\":16.000"),std::string::npos);
    EXPECT_NE(text.find("\"seen\":16.000"),std::string::npos);
    EXPECT_GT(output.WriteJSON(0,json,sizeof(json),20500,adsb),0);
    uint8_t bytes[64]; EXPECT_EQ(output.WriteGDL90(0,bytes,sizeof(bytes),20500,adsb),0);
    EXPECT_GT(output.WriteJSON(0,json,sizeof(json),64499,adsb),0);
    EXPECT_EQ(output.WriteJSON(0,json,sizeof(json),64500,adsb),0);
    char small[12]; memset(small,'!',sizeof(small));
    EXPECT_EQ(output.WriteJSON(0,small,11,20500,adsb),-1);
    EXPECT_EQ(small[0],0); EXPECT_EQ(small[11],'!');
}

TEST(Flarm, MapBatchesFilterPrivateTargetsAndCompleteEmptySnapshots) {
    FlarmTraffic traffic; FampFixture f; f.type=FAMP_ADDR_FLARM;
    for (unsigned i=0; i<64; ++i) {
        f.address=i+1; f.privacy=i%4;
        ASSERT_TRUE(traffic.Ingest(f.Packet(i+1),1000,uint64_t(f.utc)*1000,true,0,0));
    }
    FlarmSnapshot snapshot; traffic.MakeSnapshot(snapshot,1000);
    FlarmReports output; output.Update(snapshot,1000); AircraftDictionary adsb;
    char buf[513]; buf[512]='!'; std::vector<std::string> arrays;
    auto emit=[&](const char* p,uint16_t n) { arrays.emplace_back(p,n); };
    WriteAircraftJSONLiveMapArrays(adsb,1000,buf,512,emit,&output);
    size_t records=0;
    for (const auto& a : arrays) {
        EXPECT_LE(a.size(),512u); EXPECT_EQ(a.front(),'['); EXPECT_EQ(a.back(),']');
        for (size_t pos=0; (pos=a.find("\"type\":\"flarm\"",pos))!=std::string::npos; ++pos) ++records;
    }
    EXPECT_EQ(records,16u); EXPECT_EQ(buf[512],'!');
    arrays.clear();
    WriteAircraftJSONLiveMapArrays(adsb,61000,buf,512,emit,&output);
    EXPECT_TRUE(arrays.empty());
}

TEST(Flarm, MapMergesSuppressedModeSFieldsIntoFlarmRecord) {
    FlarmTraffic traffic; FampFixture f; f.type=FAMP_ADDR_ICAO; f.address=0xADF003;
    ASSERT_TRUE(traffic.Ingest(f.Packet(1),20000,uint64_t(f.utc)*1000,true,0,0));
    FlarmSnapshot snapshot; traffic.MakeSnapshot(snapshot,20000);
    FlarmReports output; output.Update(snapshot,20000); AircraftDictionary adsb;
    ModeSAircraft ac(f.address); strcpy(ac.callsign,"DKABC"); ac.squawk=7000;
    ac.WriteBitFlag(ModeSAircraft::kBitFlagBaroAltitudeValid,true); ac.baro_altitude_ft=3500;
    ac.emitter_category=ADSBTypes::kEmitterCategoryGliderSailplane;
    ASSERT_NE(adsb.InsertAircraft(ac),nullptr);
    char buf[1024]; std::string text;
    auto emit=[&](const char* p,uint16_t n) { text.append(p,n); };
    WriteAircraftJSONLiveMapArrays(adsb,21000,buf,sizeof(buf),emit,&output);
    EXPECT_EQ(text.find("\"link\":\"1090\""),std::string::npos) << text;
    EXPECT_NE(text.find("\"type\":\"flarm\""),std::string::npos) << text;
    EXPECT_NE(text.find("\"flight\":\"DKABC\""),std::string::npos) << text;
    EXPECT_NE(text.find("\"squawk\":\"7000\""),std::string::npos) << text;
    EXPECT_NE(text.find("\"alt_baro\":3500"),std::string::npos) << text;
    EXPECT_NE(text.find("\"category\":\"B1\""),std::string::npos) << text;
    EXPECT_EQ(text.find("on_ground"),std::string::npos) << text;
    EXPECT_EQ(text.back(),']');
}

TEST(Flarm, MapChoosesOneICAOSourceIndependentlyOfGDL90EmissionAge) {
    FlarmTraffic traffic; FampFixture f; f.type=FAMP_ADDR_ICAO; f.address=0xADF001;
    ASSERT_TRUE(traffic.Ingest(f.Packet(1),20000,uint64_t(f.utc)*1000,true,0,0));
    FlarmSnapshot snapshot; traffic.MakeSnapshot(snapshot,20000);
    FlarmReports output; output.Update(snapshot,20000); AircraftDictionary adsb;
    ModeSAircraft ac(f.address); ac.WriteBitFlag(ModeSAircraft::kBitFlagPositionValid,true);
    ac.last_position_update_ms=20000; ASSERT_NE(adsb.InsertAircraft(ac),nullptr);
    char buf[1024]; std::string text;
    auto emit=[&](const char* p,uint16_t n) { text.append(p,n); };
    WriteAircraftJSONLiveMapArrays(adsb,28000,buf,sizeof(buf),emit,&output);
    EXPECT_EQ(text.find("\"type\":\"flarm\""),std::string::npos);
    EXPECT_NE(text.find("\"link\":\"1090\""),std::string::npos);
    adsb.GetAircraftPtr<ModeSAircraft>(f.address)->last_position_update_ms=1000;
    text.clear(); WriteAircraftJSONLiveMapArrays(adsb,28000,buf,sizeof(buf),emit,&output);
    EXPECT_NE(text.find("\"hex\":\"adf001\""),std::string::npos);
    EXPECT_NE(text.find("\"type\":\"flarm\""),std::string::npos);
    EXPECT_EQ(text.find("\"link\""),std::string::npos);
}
