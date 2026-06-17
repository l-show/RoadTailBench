from setuptools import find_packages, setup


setup(
    name="leaderboard",
    version="0.1.0",
    description="Leaderboard closed-loop code-scenario evaluation toolkit for CARLA.",
    packages=find_packages(include=["leaderboard", "leaderboard.*"]),
    python_requires=">=3.8",
    entry_points={
        "console_scripts": [
            "leaderboard-run=leaderboard.cli.run:main",
            "leaderboard-eval=leaderboard.cli.eval:main",
            "leaderboard-plot=leaderboard.cli.plot_run:main",
        ],
    },
)
