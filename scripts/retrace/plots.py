import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def plan_view(truth_xy, est_dict, path):
    plt.figure()
    plt.plot(truth_xy[:, 0], truth_xy[:, 1], label="gps-truth")
    for k, v in est_dict.items():
        plt.plot(v[:, 0], v[:, 1], label=k)
    plt.axis("equal")
    plt.legend()
    plt.savefig(path)
    plt.close()


def error_vs_time(t, err_dict, path):
    plt.figure()
    for k, v in err_dict.items():
        plt.plot(t, v, label=k)
    plt.xlabel("s since cutoff")
    plt.ylabel("error m")
    plt.legend()
    plt.savefig(path)
    plt.close()


def combo_bars(combos, values, path):
    plt.figure()
    plt.bar(combos, values)
    plt.ylabel("home miss m")
    plt.savefig(path)
    plt.close()
