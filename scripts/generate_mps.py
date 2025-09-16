import os
import sys
import pathlib


def run_command(command):
    print(command)
    status = os.system(command)
    if status != 0:
        sys.exit(1)
    print()


main = os.path.join(
        "install",
        "bin",
        "knapsackwithconflictssolver_mps_writer")


if __name__ == "__main__":

    pathlist = pathlib.Path("data/hifi2006/").glob('*/*');
    instance_format = "hifi2006"
    for instance_path in pathlist:

        output_path = os.path.join(
                "mps",
                str(instance_path) + ".mps")
        if not os.path.exists(os.path.dirname(output_path)):
            os.makedirs(os.path.dirname(output_path))

        command = (
                f"{main}"
                f"  --input \"{instance_path}\""
                f" --format \"{instance_format}\""
                f"  --solver highs"
                f"  --output \"{output_path}\"")
        run_command(command)
