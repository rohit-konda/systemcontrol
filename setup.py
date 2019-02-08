from setuptools import setup

setup(name='systemcontrol',
      version='0.1',
      description='Python package for developing controllers for various control systems',
      URL='https://github.gatech.edu/factslab/systemcontrol',
      author='Rohit Konda',
      author_email='rkonda6@gatech.edu',
      license='None',
      packages=['systemcontrol'],
      install_requires=[
          'numpy', 'quadprog', 'matplotlib'
      ],
      classifiers=[
        'Programming Language :: Python',
        'Programming Language :: Python :: 3.5',
      ],
      zip_safe=False)