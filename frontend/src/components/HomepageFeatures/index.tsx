import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Physical AI Integration',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default, // Will be replaced with robotics-themed SVG
    description: (
      <>
        Learn how to bridge modern generative AI with hard robotics, creating intelligent
        systems that interact with the physical world using NVIDIA's ecosystem.
      </>
    ),
  },
  {
    title: 'Complete Robotics Stack',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default, // Will be replaced with robotics-themed SVG
    description: (
      <>
        Master the complete stack from hardware setup to AI integration, including
        ROS 2, Gazebo simulation, and real-world deployment strategies.
      </>
    ),
  },
  {
    title: 'Sim-to-Real Transfer',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default, // Will be replaced with robotics-themed SVG
    description: (
      <>
        Develop skills in transferring robot behaviors from simulation to real hardware,
        with focus on humanoid robotics and autonomous systems.
      </>
    ),
  },
];

function Feature({title, Svg, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
