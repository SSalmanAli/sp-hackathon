import React from 'react';
import clsx from 'clsx';
import styles from './HomepageFeatures.module.css';

type FeatureItem = {
  title: string;
  description: JSX.Element;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Hands-On Learning',
    description: (
      <>
        Learn by building real Physical AI systems with step-by-step implementation guides
        and practical exercises that reinforce concepts.
      </>
    ),
  },
  {
    title: 'Complete Ecosystem',
    description: (
      <>
        Covers the entire Physical AI stack from ROS 2 fundamentals to advanced
        vision-language-action systems and humanoid autonomy.
      </>
    ),
  },
  {
    title: 'Builder-Focused',
    description: (
      <>
        Written in a confident, no-fluff style that gets you building systems quickly
        without getting bogged down in theory.
      </>
    ),
  },
];

function Feature({title, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): JSX.Element {
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