"""Make a colormap."""
import click
import seaborn as sns
import matplotlib.pyplot as plt


@click.command()
@click.option("-n", "--name", default=None)
@click.option("-s", "--show", is_flag=True)
def main(name, show):
    """Construct a qualitative colormap."""
    sns.set()
    palette = sns.color_palette() if name is None else sns.color_palette(name)
    print(name)

    if show:
        sns.palplot(palette)
        plt.show()

    for color in palette:
        r = int(255 * color[0])
        g = int(255 * color[1])
        b = int(255 * color[2])
        print(f"{{{r}, {g}, {b}}},")


if __name__ == "__main__":
    main()
