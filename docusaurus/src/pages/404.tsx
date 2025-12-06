import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './404.module.css';

export default function NotFound(): JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout title={`Page Not Found | ${siteConfig.title}`} description="Page not found">
      <main className="container margin-vert--xl">
        <div className={styles.gridBlock}>
          <div className="text--center">
            <h1 className={styles.title}>404</h1>
            <p className={styles.subtitle}>Page Not Found</p>
            <p>We could not find what you were looking for.</p>
            <p>
              <Link to="/" className={styles.button}>
                Take me to the Physical AI Book
              </Link>
            </p>
          </div>
        </div>
      </main>
    </Layout>
  );
}