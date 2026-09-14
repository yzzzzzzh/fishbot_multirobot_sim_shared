// Probe: does pcl::KdTreeFLANN + radiusSearch SIGFPE on degenerate high-intensity
// clouds, exactly as ClusterExtractHighIntensity/FEC feed it during cold-start?
// Each case runs in a child process so one crash doesn't stop the others.
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <sys/wait.h>
#include <unistd.h>
#include <cstdio>
#include <vector>

static void build_and_search(pcl::PointCloud<pcl::PointXYZ>::Ptr c) {
    pcl::KdTreeFLANN<pcl::PointXYZ> kd;
    kd.setInputCloud(c);                       // FEC.h:45
    std::vector<int> idx; std::vector<float> d;
    if (!c->empty())
        kd.radiusSearch(c->points[0], 0.3, idx, d, 1000);  // FEC.h:65
}

static int run_case(const char* name, pcl::PointCloud<pcl::PointXYZ>::Ptr c) {
    pid_t pid = fork();
    if (pid == 0) { build_and_search(c); _exit(0); }
    int st = 0; waitpid(pid, &st, 0);
    if (WIFSIGNALED(st)) {
        int s = WTERMSIG(st);
        printf("  %-28s -> KILLED by signal %d %s\n", name, s,
               s == 8 ? "(SIGFPE!)" : s == 11 ? "(SIGSEGV)" : "");
        return s;
    }
    printf("  %-28s -> ok (exit %d)\n", name, WEXITSTATUS(st));
    return 0;
}

int main() {
    printf("PCL KdTreeFLANN degenerate-input probe:\n");

    auto empty = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    auto one = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    one->push_back({1,2,3});

    auto two_same = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    two_same->push_back({1,2,3}); two_same->push_back({1,2,3});

    auto many_same = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (int i = 0; i < 8; i++) many_same->push_back({1,2,3});   // all coincident

    auto collinear = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (int i = 0; i < 8; i++) collinear->push_back({(float)i,0,0}); // zero y,z extent

    run_case("empty", empty);
    run_case("single point", one);
    run_case("2 coincident", two_same);
    run_case("8 coincident (zero extent)", many_same);
    run_case("8 collinear (zero y/z var)", collinear);
    return 0;
}
